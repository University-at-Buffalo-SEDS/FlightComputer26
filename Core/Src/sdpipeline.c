/* Core/Src/sdpipeline.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"
#include "sd_vector_format.h"
#include "sd_record_format.h"
#include "sd_file_limit.h"

#define id "SD "

#if defined(__GNUC__) && !defined(__clang__)
/* FileX is shared by header and buffered writes. Keep one implementation;
 * LTO cloning/inlining this large routine costs flash without improving SD I/O. */
extern UINT _fx_file_write(FX_FILE *, VOID *, ULONG) __attribute__((noinline, noclone));
extern UINT _fxe_file_write(FX_FILE *, VOID *, ULONG) __attribute__((noinline, noclone));
#endif


static FX_FILE file = {0};
static sd_meta line = {.free = true};
static uint32_t sd_csv_sequence;
static uint32_t sd_buffer_records[2];
static bool sd_writer_slot;
static bool sd_retry_pending;
static volatile bool sd_accepting;
/* 0: closed, 1: header complete, 2: retain handle and retry partial header. */
static uint8_t sd_file_open;
volatile uint32_t g_sd_unavailable_drops;
static ULONG sd_retry_offset;
volatile uint32_t g_sd_enqueued_records;
volatile uint32_t g_sd_written_records;
volatile uint32_t g_sd_flushed_records;
volatile uint32_t g_sd_shutdown_lost_records;
volatile uint32_t g_sd_write_errors;

uncached char sdbuf[2][SD_BUFFER_SIZE] = {0};


/* Helpers */

volatile uint32_t g_sd_format_errors = 0U;
volatile uint32_t g_sd_lock_contention_drops = 0U;

fi16 seds_ftoa4(char *dst, fu16 capacity, const float *data, fu8 count)
{
    return (fi16)sd_format_vector(dst, capacity, data, count);
}

static inline void sd_release_notify(fu16 written, fu16 rem)
{
    if (written > 0 && written < rem)
    {
        line.off[line.cur] += written;
        ++sd_buffer_records[line.cur];
        ++g_sd_enqueued_records;

        if (line.off[line.cur] >= SD_POST_MARGIN)
        {
            if (line.free)
            {
                /* Never hand a producer the buffer still being written to SD. */
                line.cur = !line.cur;
                line.free = false;
                fc_unlock(&line.lock);
                tx_semaphore_put(&line.full);
                return;
            }
            /* The active buffer can retain more complete rows until full.
             * A rejected append is counted by the bounded formatter. */

            fc_unlock(&line.lock);
            return;
        }
    }
    fc_concede(&line.lock);
}

static inline constexpr char *seds_msg(SedsDataType ty)
{
    /* Local schema IDs intentionally extend the generated enum. Switching on
     * the underlying value keeps GCC from diagnosing those valid IDs. */
    switch ((int)ty) {
        case SEDS_DT_MESSAGE_DATA:		return "Message";
        case SEDS_DT_ORDERED_MESSAGE:	return "Ordered";
        case SEDS_DT_GENERIC_ERROR:		return "Error";
        default:						return "Echo";
    }
}

static inline constexpr char *seds_f32(SedsDataType ty)
{
    switch ((int)ty) {
        case SEDS_DT_IMU_LOCAL:			  return "IMU";
        case SEDS_DT_BAROMETER_LOCAL:	return "BARO";
        case SEDS_DT_ACCEL_LOCAL:		  return "ACCL";
        case SEDS_DT_GYRO_LOCAL:		  return "GYRO";
        case SEDS_DT_ASCENT_LOCAL:    return "EKF";
        case SEDS_DT_DESCENT_LOCAL:   return "DKF";
        case SEDS_DT_EULER_ANGLES:    return "EULER";
        default:						          return "QUAT";
    }
}

static inline char *unique_sd_filename(uint32_t suffix)
{
    static char fbuf[26 + sizeof SEDS_LOG_FILENAME];
    static_assert(sizeof SEDS_LOG_FILENAME == strlen(SEDS_LOG_FILENAME) + 1, "");


    const uint32_t network_time = (uint32_t)telemetry_unix_s();
    const uint32_t time = network_time != 0U ? network_time : now_ms();

    snprintf(fbuf, sizeof fbuf, SEDS_LOG_FILENAME "-%lu-%03lu.csv",
             (unsigned long)time, (unsigned long)suffix);

    return fbuf;
}


/* API */

/* All producers share one bounded append path. Clock queries precede the
 * SD lock; a preempted writer never makes a producer spin. */
#if defined(__GNUC__)
__attribute__((noinline))
#endif
static void sd_append_ready(const char *sensor, const char *value,
                            const char *raw, uint64_t uptime)
{
    const uint64_t current_network_ms = telemetry_unix_ms();
    const uint64_t age = board_uptime_ticks() - uptime;
    const uint64_t network_ms = current_network_ms > age ? current_network_ms - age : 0U;
    if (!fc_trylock(&line.lock)) { ++g_sd_lock_contention_drops; return; }
    /* Conclude may race the pre-lock check. */
    if (load(&g_conf, Acq) & option(SD_Pipeline_Reset)) {
        fc_concede(&line.lock); return;
    }
    char *off = sdbuf[line.cur] + line.off[line.cur];
    fu16 rem = SD_BUFFER_SIZE - line.off[line.cur];
    int written = sd_format_csv_common(off, rem, network_ms, uptime,
        sensor, value, raw, ++sd_csv_sequence);
    if (written < 0) { g_sd_format_errors++; fc_concede(&line.lock); return; }
    sd_release_notify(written, rem);
}

void sd_append_f32(SedsDataType ty, const float *data, fu8 count)
{
    if (!data || count == 0 ||
        (load(&g_conf, Acq) & option(SD_Pipeline_Reset))) return;
    if (!sd_accepting) { ++g_sd_unavailable_drops; return; }
    char buf[F32_TO_STR_MAX_SIZE];
    if (seds_ftoa4(buf, sizeof(buf), data, count) < 0) {
        g_sd_format_errors++; return;
    }
    sd_append_ready(seds_f32(ty), buf, NULL, board_uptime_ticks());
}

/* Called by the acquisition consumer, never the DMA interrupt. Independent
 * per-sensor SD cadence does not consume the existing IMU/barometer timers. */
void sd_append_sensor(unsigned sensor, const float *raw, const float *converted, fu8 count)
{
    static uint32_t last_ms[3];
    static const char *const names[3] = {"barometer_sample", "gyroscope_sample", "accelerometer_sample"};
    if (sensor >= 3U || !raw || !converted || count == 0U || count > 3U ||
        (load(&g_conf, Acq) & option(SD_Pipeline_Reset))) return;
    const uint64_t uptime = board_uptime_ticks();
    if ((uint32_t)((uint32_t)uptime - last_ms[sensor]) < rates.sd) return;
    last_ms[sensor] = (uint32_t)uptime;
    if (!sd_accepting) { ++g_sd_unavailable_drops; return; }
    char raw_text[80], converted_text[80];
    if (sd_format_vector(raw_text, sizeof(raw_text), raw, count) < 0 ||
        sd_format_vector(converted_text, sizeof(converted_text), converted, count) < 0) {
        g_sd_format_errors++; return;
    }
    sd_append_ready(names[sensor], converted_text, raw_text, uptime);
}

void sd_append_string(SedsDataType ty, const char *str)
{
    if (!str || (load(&g_conf, Acq) & option(SD_Pipeline_Reset))) return;
    if (!sd_accepting) { ++g_sd_unavailable_drops; return; }
    sd_append_ready(seds_msg(ty), str, NULL, board_uptime_ticks());
}

void sd_conclude(void)
{
    /* Reset is the producer stop flag; the writer owns buffer bookkeeping.
     * Do not spin here from the higher-priority recovery task. */
    fetch_or(&g_conf, option(SD_Pipeline_Reset), Rel);
    tx_semaphore_put(&line.full);
}


/* Task */

/* Startup, file-part creation and shutdown are I/O-bound cold paths. Keep
 * the normal producer/writer paths optimized independently for steady traffic. */
static inline __attribute__((cold)) int sd_pipeline_open(const char *surprise);
static inline __attribute__((cold)) int sd_pipeline_init(const char *surprise);
static inline __attribute__((cold)) void sd_pipeline_shutdown(const char *unluck);

static inline bool sd_try_write_line(void)
{
    fc_lock(&line.lock);
    const fu16 write_len = line.off[sd_writer_slot];
    fc_unlock(&line.lock);
    if (write_len == 0U) return true;

    /* A failed FileX write can advance the file offset partially. Retain both
     * buffer ownership and its original offset until the complete retry wins. */
    if (!sd_retry_pending)
    {
        if (sd_file_open && sd_log_part_needs_rotation(file.fx_file_current_file_size, write_len))
        {
            if (fx_media_flush(&sdio_disk) != FX_SUCCESS || fx_file_close(&file) != FX_SUCCESS)
            { ++g_sd_write_errors; return false; }
            g_sd_flushed_records = g_sd_written_records;
            sd_file_open = false;
        }
        if (sd_file_open != 1U && sd_pipeline_open("part:") != WE_ARE_SO_BACK)
        { ++g_sd_write_errors; return false; }
        sd_retry_offset = file.fx_file_current_file_size;
        sd_retry_pending = true;
    }
    sweetbench_start(12);
    UINT st = fx_file_seek(&file, sd_retry_offset);
    if (st == FX_SUCCESS)
        st = fx_file_write(&file, sdbuf[sd_writer_slot], write_len);
    sweetbench_catch(12);
    if (st != FX_SUCCESS) { ++g_sd_write_errors; return false; }

    fc_lock(&line.lock);
    g_sd_written_records += sd_buffer_records[sd_writer_slot];
    sd_buffer_records[sd_writer_slot] = 0U;
    line.off[sd_writer_slot] = 0U;
    line.free = true;
    sd_writer_slot = !sd_writer_slot;
    sd_retry_pending = false;
    fc_unlock(&line.lock);
    return true;
}

static inline void sd_pipeline_shutdown(const char *unluck)
{
    /* Producers recheck the stop flag under the lock. Drain the handed-off
     * buffer first, then the active tail, even when it never reached margin. */
    sd_accepting = false;
    for (unsigned slot = 0U; slot < 2U; ++slot)
    {
        /* Synchronize with a producer already formatting when stop was set.
         * An unlocked empty check could skip its just-committed final row. */
        fc_lock(&line.lock);
        const bool empty = line.off[sd_writer_slot] == 0U;
        fc_unlock(&line.lock);
        if (empty) { sd_writer_slot = !sd_writer_slot; continue; }
        unsigned attempts = 0U;
        while (!sd_try_write_line())
        {
            if (++attempts == 3U) break;
            tx_thread_sleep(1U);
        }
        if (attempts == 3U) break;
    }
    g_sd_shutdown_lost_records += sd_buffer_records[0] + sd_buffer_records[1];
    UINT st = fx_media_flush(&sdio_disk);
    if (st == FX_SUCCESS) g_sd_flushed_records = g_sd_written_records;
    else ++g_sd_write_errors;
    if (sd_file_open && (st = fx_file_close(&file)) != FX_SUCCESS)
        log_err(id "fclose %s %u", unluck, st);
    sd_file_open = false;
    if ((st = fx_media_close(&sdio_disk)) != FX_SUCCESS)
        log_err(id "mclose %s %u", unluck, st);
}

static inline int sd_pipeline_init(const char *surprise)
{
    sd_accepting = false;
    fetch_and(&g_conf, ~option(SD_Pipeline_Reset), Rlx);

    memset(&line, 0, sizeof line);
    line.free = true;
    sd_buffer_records[0] = sd_buffer_records[1] = 0U;
    sd_writer_slot = false;
    sd_retry_pending = false;
    sd_file_open = false;

    UINT st = tx_semaphore_create(&line.full, id "S", 0);

    if (st != TX_SUCCESS)
    {
        log_err(id "sema %s %u", surprise, st);
        return IT_IS_NOW_OVER;
    }
    for (;;)
    {
        if (load(&g_conf, Acq) & option(SD_Pipeline_Reset))
        {
            sd_pipeline_shutdown(surprise);
            /* The static semaphore retains its normal boot-long lifetime. */
            return IT_IS_NOW_OVER;
        }
        if (sd_pipeline_open(surprise) == WE_ARE_SO_BACK)
        {
            sd_accepting = true;
            return WE_ARE_SO_BACK;
        }
        ++g_sd_write_errors;
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
    }
}

static inline int sd_pipeline_open(const char *surprise)
{
    UINT st;
    if (!sd_file_open)
    {
        for (uint32_t suffix = 0U; suffix < 1000U; ++suffix)
        {
            file.fx_file_name = unique_sd_filename(suffix);
            st = fx_file_create(&sdio_disk, file.fx_file_name);
            if (st != FX_ALREADY_CREATED) break;
        }
        if (st != FX_SUCCESS)
        {
            if (!sd_accepting) log_err(id "fcreate %s %u", surprise, st);
            return IT_IS_NOW_OVER;
        }
        st = fx_file_open(&sdio_disk, &file, file.fx_file_name, FX_OPEN_FOR_WRITE);
        if (st != FX_SUCCESS)
        {
            if (!sd_accepting) log_err(id "fopen %s %u", surprise, st);
            return IT_IS_NOW_OVER;
        }
        sd_file_open = 2U;
    }
    if (sd_file_open == 2U)
    {
        static const char header[] = SD_CSV_HEADER;
        st = fx_file_seek(&file, 0U);
        if (st == FX_SUCCESS) st = fx_file_write(&file, (void *)header, sizeof(header) - 1U);
        if (st != FX_SUCCESS) return IT_IS_NOW_OVER;
        sd_file_open = 1U;
    }
    return WE_ARE_SO_BACK;
}

void sd_pipeline_task()
{
    UINT st;
    const char *surprise = "failed:";

    if (sd_pipeline_init(surprise) != WE_ARE_SO_BACK)
    {
        return;
    }

    bool pending_write = false;
    MrAnalog (load(&g_conf, Acq) & option(SD_Pipeline_Reset))
    {
        if (!pending_write && tx_semaphore_get(&line.full, TX_WAIT_FOREVER) != TX_SUCCESS)
            continue;
        if (load(&g_conf, Acq) & option(SD_Pipeline_Reset)) break;
        if (!sd_try_write_line())
        {
            pending_write = true;
            /* Retry retained data without waiting for another producer signal.
             * Yield so acquisition and networking continue during card stalls. */
            tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND + 19U) / 20U);
            continue;
        }
        pending_write = false;
        if (timer_probe(SDFlush, SD_FLUSH_INTERVAL))
        {
            st = fx_media_flush(&sdio_disk);
            if (st == FX_SUCCESS) g_sd_flushed_records = g_sd_written_records;
            else { ++g_sd_write_errors; log_err(id "flush %s %u", surprise, st); }
        }
    }
    sd_pipeline_shutdown(surprise);
}
