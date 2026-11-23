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
#include "fc_sd_events.h"
#include "resilient_storage.h"
#include "ground_test.h"
#ifdef FC_SD_SERVICE
#include "sd_file_stream.h"
#include "launchcore/image.h"
#endif

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
static bool sd_launch_pending, sd_launch_active;
static uint32_t sd_buffer_records[2];
/* One bounded file boundary, retained across partial writes and close failures. */
static volatile bool sd_landed_seen;
static bool sd_launch_completed, sd_cutoff_set, sd_cutoff_slot;
static volatile uint32_t sd_landed_tick;
static uint32_t sd_cutoff_records;
static fu16 sd_cutoff_offset, sd_writer_skip;
volatile uint32_t g_sd_launch_closed;

void sd_launch_landed(uint32_t tick)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!sd_landed_seen) { sd_landed_tick = tick; sd_landed_seen = true; }
    __set_PRIMASK(mask);
}

/* Caller owns line.lock. UTC changes and later Recovery do not move the timer.
 * Producers only mark an offset; the SD worker alone touches FileX. */
static void sd_mark_launch_cutoff(void)
{
    if (sd_landed_seen && !sd_launch_completed && !sd_cutoff_set &&
        (sd_launch_active || sd_launch_pending) &&
        (uint32_t)(now_ms() - sd_landed_tick) >= 20000U) {
        sd_cutoff_slot = line.cur;
        sd_cutoff_offset = line.off[line.cur];
        sd_cutoff_records = sd_buffer_records[line.cur];
        sd_cutoff_set = true;
    }
}

/* Logging continues through Recovery until power-off, bounded: an hour after
 * Landed (once the launch file has closed) the writer stops, flushes and, with
 * FC_SD_SERVICE, keeps serving reads. Roughly 10-20 MB of descent-rate rows. */
#define SD_RECOVERY_LOG_MS (60U * 60U * 1000U)
static bool sd_recovery_log_expired(void)
{
    return sd_landed_seen && !sd_launch_active && !sd_launch_pending && !sd_cutoff_set &&
           (uint32_t)(now_ms() - sd_landed_tick) >= SD_RECOVERY_LOG_MS;
}
static bool sd_writer_slot;
static bool sd_retry_pending;
static volatile bool sd_accepting;
static bool sd_prepared;
/* 0: closed, 1: header complete, 2: retain handle and retry partial header. */
static uint8_t sd_file_open;
volatile uint32_t g_sd_unavailable_drops;
static ULONG sd_retry_offset;
volatile uint32_t g_sd_enqueued_records;
volatile uint32_t g_sd_written_records;
volatile uint32_t g_sd_flushed_records;
volatile uint32_t g_sd_shutdown_lost_records;
volatile uint32_t g_sd_shutdown_lost_events;
volatile uint32_t g_sd_write_errors;
volatile uint32_t g_sd_max_write_ms;

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
#if FC_GROUND_TEST
        case SEDS_DT_SIM_SAMPLE:      return "SIM";
#endif
        default:						          return "QUAT";
    }
}

static inline char *unique_sd_filename(uint32_t suffix)
{
    static char fbuf[34 + sizeof SEDS_LOG_FILENAME];
    static_assert(sizeof SEDS_LOG_FILENAME == strlen(SEDS_LOG_FILENAME) + 1, "");


    const uint32_t network_time = (uint32_t)telemetry_unix_s();
    const uint32_t time = network_time != 0U ? network_time : now_ms();

    snprintf(fbuf, sizeof fbuf, "%s%s-%lu-%03lu.csv",
             SEDS_LOG_FILENAME, sd_launch_active ? "_LAUNCH" : "",
             (unsigned long)time, (unsigned long)suffix);

    return fbuf;
}


/* API */

/* Only the SD worker queries the network clock. Producers take a bounded
 * snapshot; a router mutex or blocked card cannot hold up flight acquisition.
 * Expire the UTC lease after two seconds without a worker refresh. Rows then
 * retain their exact independent uptime and explicitly use time_source=local. */
static uint64_t sd_clock_utc_ms, sd_clock_uptime;
static void sd_refresh_clock(void)
{
#ifdef TELEMETRY_ENABLED
    const uint64_t utc = telemetry_unix_ms();
#else
    const uint64_t utc = 0U;
#endif
    const uint64_t uptime = board_uptime_ticks();
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    sd_clock_utc_ms = utc;
    sd_clock_uptime = uptime;
    __set_PRIMASK(mask);
}

static uint64_t sd_record_utc(uint64_t captured)
{
    const uint64_t now = board_uptime_ticks();
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint64_t utc = sd_clock_utc_ms, anchor = sd_clock_uptime;
    __set_PRIMASK(mask);
    /* Do not extrapolate before this known clock mapping (e.g. an event queued
     * before first sync or across a UTC correction). Never manufacture UTC. */
    return utc && now >= anchor && now - anchor <= 2000U && captured >= anchor &&
           UINT64_MAX - utc >= captured - anchor ? utc + captured - anchor : 0U;
}

/* All producers share one bounded append path; no filesystem or network calls
 * and a busy SD lock rejects an append immediately instead of spinning. */
#if defined(__GNUC__)
__attribute__((noinline))
#endif
static bool sd_append_ready(const char *sensor, const char *value,
                            const char *raw, uint64_t uptime, bool draining)
{
    const uint64_t network_ms = sd_record_utc(uptime);
    if (!fc_trylock(&line.lock)) { ++g_sd_lock_contention_drops; return false; }
    /* Conclude may race the pre-lock check. */
    if (!draining && (load(&g_conf, Acq) & option(SD_Pipeline_Reset))) {
        fc_concede(&line.lock); return false;
    }
    sd_mark_launch_cutoff();
    char *off = sdbuf[line.cur] + line.off[line.cur];
    fu16 rem = SD_BUFFER_SIZE - line.off[line.cur];
    int written = sd_format_csv_common(off, rem, network_ms, uptime,
        sensor, value, raw, ++sd_csv_sequence);
    if (written < 0) { g_sd_format_errors++; fc_concede(&line.lock); return false; }
    sd_release_notify(written, rem);
    return true;
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
    sd_append_ready(seds_f32(ty), buf, NULL, board_uptime_ticks(), false);
}

/* Called once per consumed acquisition, never the DMA interrupt. Raw samples
 * keep their full acquired cadence independently of network/Kalman reports. */
void sd_append_sensor(unsigned sensor, const float *raw, const float *converted, fu8 count)
{
    static const char *const names[3] = {"barometer_sample", "gyroscope_sample", "accelerometer_sample"};
    if (sensor >= 3U || !raw || !converted || count == 0U || count > 3U ||
        (load(&g_conf, Acq) & option(SD_Pipeline_Reset))) return;
    const uint64_t uptime = board_uptime_ticks();
    if (!sd_accepting) { ++g_sd_unavailable_drops; return; }
    char raw_text[80], converted_text[80];
    if (sd_format_vector(raw_text, sizeof(raw_text), raw, count) < 0 ||
        sd_format_vector(converted_text, sizeof(converted_text), converted, count) < 0) {
        g_sd_format_errors++; return;
    }
    sd_append_ready(names[sensor], converted_text, raw_text, uptime, false);
}

void sd_append_string(SedsDataType ty, const char *str)
{
    if (!str || (load(&g_conf, Acq) & option(SD_Pipeline_Reset))) return;
    if (!sd_accepting) { ++g_sd_unavailable_drops; return; }
    sd_append_ready(seds_msg(ty), str, NULL, board_uptime_ticks(), false);
}

void sd_conclude(void)
{
    /* Reset is the producer stop flag; the writer owns buffer bookkeeping.
     * Do not spin here from the higher-priority recovery task. */
    fetch_or(&g_conf, option(SD_Pipeline_Reset), Rel);
    tx_semaphore_put(&line.full);
}


/* Preserve FIFO event ownership until the complete CSV row is buffered.
 * Producer capture never waits for this lower-priority task or the card. */
static void sd_service_events(bool draining)
{
    static const char *const labels[] = {
        "launch_detected", "pyro_gpio_high", "pyro_gpio_low",
        "pyro_rejected", "pyro_already_active", "launch_accepted",
        "pyro_dry_run"
    };
    fc_sd_event event;
    for (unsigned n = 0U; n < 32U && fc_sd_peek_event(&event); ++n)
    {
        const uint64_t uptime = board_uptime_project(board_uptime_ticks(), event.tick);
        const bool launch = event.kind == FC_LOG_LAUNCH_DETECTED || event.kind == FC_LOG_LAUNCH_ACCEPTED;
        if (!sd_append_ready(labels[event.kind], launch ? "Launch" :
                             (event.reef ? "reef" : "drogue"), NULL, uptime, draining))
            break;
        if (launch && !sd_launch_active && !sd_cutoff_set &&
            (!sd_launch_completed || (current() < Landed &&
             (uint32_t)(event.tick - sd_landed_tick) < UINT32_C(0x80000000)))) {
            if (sd_launch_completed) sd_landed_seen = false;
            sd_launch_completed = false;
            sd_launch_pending = true;
        }
        fc_sd_consume_event();
    }
}

/* Called only by the writer. Transfer a short active tail using exactly the
 * same ownership protocol as the full-buffer path. Never expose in-flight bytes
 * to a producer, including after a partially failed FileX write. */
static void sd_handoff_tail(void)
{
    fc_lock(&line.lock);
    if (line.free && line.off[line.cur] != 0U) {
        line.cur = !line.cur;
        line.free = false;
    }
    fc_unlock(&line.lock);
}

/* Task */

/* Called during ThreadX initialization, before acquisition tasks run. Card
 * mounting must not be a prerequisite for bounded startup admission. */
UINT sd_pipeline_prepare(void)
{
    if (sd_prepared) return TX_SUCCESS;
    sd_accepting = false;
    fetch_and(&g_conf, ~option(SD_Pipeline_Reset), Rlx);

    memset(&line, 0, sizeof line);
    line.free = true;
    sd_buffer_records[0] = sd_buffer_records[1] = 0U;
    sd_writer_slot = false;
    sd_retry_pending = false;
    sd_file_open = false;
    sd_launch_pending = false;

    UINT st = tx_semaphore_create(&line.full, id "S", 0);

    if (st == TX_SUCCESS) {
        sd_prepared = true;
        sd_accepting = true;
    }
    return st;
}

/* Startup, file-part creation and shutdown are I/O-bound cold paths. Keep
 * the normal producer/writer paths optimized independently for steady traffic. */
static inline __attribute__((cold)) int sd_pipeline_open(const char *surprise);
static inline __attribute__((cold)) int sd_pipeline_init(const char *surprise);
static inline __attribute__((cold)) void sd_pipeline_shutdown(const char *unluck);

#ifdef FC_SD_SERVICE
static bool dl_open, dl_list_valid;
#endif

static inline bool sd_io_failed(UINT status)
{
    if (status == FX_SUCCESS) {
        if (flight_sd_note_progress()) return false;
        ++g_sd_write_errors; return true; /* retain row/offset; no fake FX status */
    }
    ++g_sd_write_errors;
    flight_sd_note_io_failure(status);
    return true;
}
static inline bool sd_flush_close_current(void)
{
    if (sd_io_failed(fx_media_flush(&sdio_disk))) return false;
    const UINT closed = fx_file_close(&file);
    if (closed == FX_SUCCESS) sd_file_open = 0U; /* actual logical handle state */
    if (sd_io_failed(closed)) return false;
    g_sd_flushed_records = g_sd_written_records;
    return true;
}

static inline bool sd_try_write_line(void)
{
    if (flight_sd_prepare_io() != FX_SUCCESS) return false;
resume_segment:
    fc_lock(&line.lock);
    const fu16 total = line.off[sd_writer_slot];
    const bool boundary = sd_cutoff_set && sd_cutoff_slot == sd_writer_slot;
    const fu16 end = boundary ? sd_cutoff_offset : total;
    const fu16 write_len = end - sd_writer_skip;
    fc_unlock(&line.lock);
    if (boundary && write_len == 0U && !sd_retry_pending) {
        if (sd_file_open == 2U && sd_pipeline_open("landed:") != WE_ARE_SO_BACK)
        { ++g_sd_write_errors; return false; }
        if (sd_file_open && !sd_flush_close_current()) return false;
        sd_file_open = 0U;
        sd_launch_active = sd_launch_pending = false;
        sd_launch_completed = true;
        sd_cutoff_set = false;
        ++g_sd_launch_closed;
        goto resume_segment;
    }
    if (write_len == 0U) {
        if (sd_writer_skip != 0U) {
            fc_lock(&line.lock);
            line.off[sd_writer_slot] = 0U;
            line.free = true;
            sd_writer_slot = !sd_writer_slot;
            sd_writer_skip = 0U;
            fc_unlock(&line.lock);
        }
        return true;
    }

    /* A failed FileX write can advance the file offset partially. Retain both
     * buffer ownership and its original offset until the complete retry wins. */
    if (!sd_retry_pending)
    {
        if (sd_file_open && (sd_launch_pending ||
            sd_log_part_needs_rotation(file.fx_file_current_file_size, write_len)))
        {
            if (!sd_flush_close_current()) return false;
            sd_file_open = false;
        }
        if (sd_launch_pending) { sd_launch_active = true; sd_launch_pending = false; }
        if (sd_file_open != 1U && sd_pipeline_open("part:") != WE_ARE_SO_BACK)
        { ++g_sd_write_errors; return false; }
        sd_retry_offset = file.fx_file_current_file_size;
        sd_retry_pending = true;
    }
    const uint32_t write_started = now_ms();
    sweetbench_start(12);
    UINT st = fx_file_seek(&file, sd_retry_offset);
    if (st == FX_SUCCESS && !flight_sd_io_verified()) { ++g_sd_write_errors; return false; }
    if (st == FX_SUCCESS)
        st = fx_file_write(&file, sdbuf[sd_writer_slot] + sd_writer_skip, write_len);
    sweetbench_catch(12);
    const uint32_t write_duration = (uint32_t)(now_ms() - write_started);
    if (write_duration > g_sd_max_write_ms) g_sd_max_write_ms = write_duration;
    if (sd_io_failed(st)) return false;

    fc_lock(&line.lock);
    if (boundary) {
        g_sd_written_records += sd_cutoff_records;
        sd_buffer_records[sd_writer_slot] -= sd_cutoff_records;
        sd_writer_skip = end;
        sd_retry_pending = false;
        fc_unlock(&line.lock);
        goto resume_segment;
    }
    g_sd_written_records += sd_buffer_records[sd_writer_slot];
    sd_buffer_records[sd_writer_slot] = 0U;
    line.off[sd_writer_slot] = 0U;
    line.free = true;
    sd_writer_slot = !sd_writer_slot;
    sd_retry_pending = false;
    sd_writer_skip = 0U;
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
    /* After raw tails drain, reserve their now-free space for queued audit
     * events. Only this writer may append after the producer stop flag. */
    if (sd_buffer_records[0] == 0U && sd_buffer_records[1] == 0U) {
        line.cur = sd_writer_slot;
        sd_service_events(true);
        sd_handoff_tail();
        for (unsigned attempt = 0U; !line.free && attempt < 3U; ++attempt) {
            if (sd_try_write_line()) break;
            if (attempt < 2U) tx_thread_sleep(1U);
        }
    }
    g_sd_shutdown_lost_events += g_sd_event_pending;
    g_sd_shutdown_lost_records += sd_buffer_records[0] + sd_buffer_records[1];
    /* Do not issue another FileX operation after a transport failure until
     * the retained card's identity and DMA ownership have been re-established.
     * An explicit stop remains bounded; failed close leaves its handle owned. */
    if (flight_sd_prepare_io() != FX_SUCCESS) return;
    UINT st = fx_media_flush(&sdio_disk);
    if (sd_io_failed(st)) return;
    g_sd_flushed_records = g_sd_written_records;
    if (sd_file_open) {
        st = fx_file_close(&file);
        if (st == FX_SUCCESS) sd_file_open = false;
        if (sd_io_failed(st)) { log_err(id "fclose %s %u", unluck, st); return; }
    }
    sd_file_open = false;
#ifndef FC_SD_SERVICE
    st = fx_media_close(&sdio_disk);
    if (sd_io_failed(st)) log_err(id "mclose %s %u", unluck, st);
#else
    /* fx_file_close() writes the final directory entry (file size) into the
     * media cache; without fx_media_close() nothing else writes it out, so
     * flush again or an AV-bay power-off could leave the size stale. */
    st = fx_media_flush(&sdio_disk);
    if (sd_io_failed(st)) log_err(id "mflush %s %u", unluck, st);
#endif
    /* With the read service the flushed volume stays mounted, read-only from
     * here on; flight_sd_prepare_io() re-initialises a lost transport. */
}

/* The card left its slot and a fresh mount replaces the old one: the log
 * handle is aborted, so the retained row goes whole into a new log part. */
void sd_pipeline_media_remounted(void)
{
    sd_file_open = 0U;
    sd_retry_pending = false;
#ifdef FC_SD_SERVICE
    sd_file_stream_handle_open(dl_open = dl_list_valid = false);
#endif
}

static inline int sd_pipeline_init(const char *surprise)
{
    UINT st = sd_pipeline_prepare();
    if (st != TX_SUCCESS) {
        log_err(id "sema %s %u", surprise, st);
        return IT_IS_NOW_OVER;
    }
    sd_launch_active = beyond(Armed) && !sd_launch_completed;
    if (current() >= Landed && !sd_landed_seen) sd_launch_landed(now_ms());
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
            g_fc_sd_state = 0U;
            return WE_ARE_SO_BACK;
        }
        /* This loop already retries every second: an operator retry is taken. */
        g_fc_sd_state = FC_SD_NOT_READY;
        ++g_sd_write_errors;
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
    }
}

static inline int sd_pipeline_open(const char *surprise)
{
    if (flight_sd_prepare_io() != FX_SUCCESS) return IT_IS_NOW_OVER;
    UINT st;
    if (!sd_file_open)
    {
        for (uint32_t suffix = 0U; suffix < 1000U; ++suffix)
        {
            file.fx_file_name = unique_sd_filename(suffix);
            st = fx_file_create(&sdio_disk, file.fx_file_name);
            if (st != FX_ALREADY_CREATED) break;
            if (!flight_sd_io_verified()) { flight_sd_note_io_failure(st); return IT_IS_NOW_OVER; }
        }
        if (st != FX_SUCCESS)
        {
            flight_sd_note_io_failure(st);
            if (!sd_accepting) log_err(id "fcreate %s %u", surprise, st);
            return IT_IS_NOW_OVER;
        }
        if (!flight_sd_io_verified()) return IT_IS_NOW_OVER;
        st = fx_file_open(&sdio_disk, &file, file.fx_file_name, FX_OPEN_FOR_WRITE);
        if (st != FX_SUCCESS)
        {
            flight_sd_note_io_failure(st);
            if (!sd_accepting) log_err(id "fopen %s %u", surprise, st);
            return IT_IS_NOW_OVER;
        }
        sd_file_open = 2U;
        if (!flight_sd_io_verified()) return IT_IS_NOW_OVER;
    }
    if (sd_file_open == 2U)
    {
        static const char header[] = SD_FILE_HEADER;
        static_assert(BOARD_UPTIME_FREQUENCY_HZ == 1000U, "SD_FORMAT_BLOCK tick_hz");
        st = fx_file_seek(&file, 0U);
        if (st == FX_SUCCESS && !flight_sd_io_verified()) return IT_IS_NOW_OVER;
        if (st == FX_SUCCESS) st = fx_file_write(&file, (void *)header, sizeof(header) - 1U);
        if (sd_io_failed(st)) return IT_IS_NOW_OVER;
        sd_file_open = 1U;
    }
    return WE_ARE_SO_BACK;
}

#ifdef FC_SD_SERVICE
/* Network SD transfer service v2 (sd_file_stream.c, sd_xfer.c), in this
 * thread between writer passes and, after sd_conclude, in a read-only loop.
 * Each turn does at most one bounded step: a directory step, one DATA message
 * (at most one block re-encoded), or a hash step. The read handle is a
 * separate FX_OPEN_FOR_READ owner in the borrowed area; it never writes. */
#define SD_DL_IDLE_CLOSE_MS 3000U
#define SD_DL_LIST_SCAN_PER_TURN 32U
#define SD_DL_HASH_SECTORS_PER_TURN 8U
#define dl_file (*(FX_FILE *)sd_file_stream_handle())
static uint16_t dl_list_next;
static ULONG dl_list_entry;
static uint32_t dl_last_ms;
static struct { bool active, growing; uint32_t ticket, pos, size; } dl_hash;
volatile uint32_t g_sd_dl_errors;

void sd_pipeline_wake(void)
{
    if (sd_prepared) (void)tx_semaphore_put(&line.full);
}

static void dl_close(void)
{
    if (dl_open) { (void)fx_file_close(&dl_file); dl_open = false; sd_file_stream_handle_open(false); }
}

static bool dl_growing(const char *name)
{
    return sd_file_open && file.fx_file_name && strcmp(file.fx_file_name, name) == 0;
}

/* LIST places the name at the start of the block. Only the iterator persists;
 * a long skip answers BUSY and continues on the station's retry. */
static uint8_t dl_list(const sd_file_request_t *rq, CHAR *name, uint32_t *size, uint8_t *flags)
{
    if (!dl_list_valid || rq->index < dl_list_next) {
        dl_list_valid = true; dl_list_next = 0U; dl_list_entry = 0U;
    }
    for (unsigned scanned = 0U; scanned < SD_DL_LIST_SCAN_PER_TURN; ++scanned) {
        UINT attributes = 0U, year, month, day, hour, minute, second;
        ULONG bytes = 0U;
        sdio_disk.fx_media_default_path.fx_path_current_entry = dl_list_entry;
        const UINT st = fx_directory_next_full_entry_find(&sdio_disk, name, &attributes, &bytes,
                                                          &year, &month, &day, &hour, &minute, &second);
        dl_list_entry = sdio_disk.fx_media_default_path.fx_path_current_entry;
        if (st == FX_NO_MORE_ENTRIES) return SD_FILE_STATUS_END;
        if (st != FX_SUCCESS || !flight_sd_io_verified()) { dl_list_valid = false; return SD_FILE_STATUS_IO_ERROR; }
        name[FX_MAX_LONG_NAME_LEN - 1U] = '\0';
        if ((attributes & (FX_DIRECTORY | FX_VOLUME | FX_HIDDEN | FX_SYSTEM)) != 0U ||
            !sd_file_fc_log_name(name, strlen(name)) || dl_list_next++ != rq->index) continue;
        *size = (uint32_t)bytes;
        *flags = dl_growing(name) ? SD_FILE_ENTRY_OPEN : 0U;
        return SD_FILE_STATUS_OK;
    }
    return SD_FILE_STATUS_BUSY;
}

/* Opens (or keeps) the read handle on name; *size is the size it sees. */
static uint8_t dl_open_file(const char *name, uint32_t *size)
{
    if (dl_open && (strcmp(name, dl_file.fx_file_name_buffer) != 0 || dl_growing(name))) dl_close();
    if (!dl_open) {
        const UINT st = fx_file_open(&sdio_disk, &dl_file, (CHAR *)name, FX_OPEN_FOR_READ);
        if (st != FX_SUCCESS) return st == FX_NOT_FOUND ? SD_FILE_STATUS_NOT_FOUND : SD_FILE_STATUS_IO_ERROR;
        dl_open = true; sd_file_stream_handle_open(true);
    }
    const ULONG64 size64 = dl_file.fx_file_current_file_size;
    *size = size64 > UINT32_MAX ? UINT32_MAX : (uint32_t)size64;
    return SD_FILE_STATUS_OK;
}

/* sd_xfer read callback: whole 512-byte sectors (or the file tail). */
static bool dl_read(void *ctx, uint32_t offset, uint8_t *dst, uint32_t len)
{
    (void)ctx;
    if (!dl_open || flight_sd_prepare_io() != FX_SUCCESS || fx_file_seek(&dl_file, offset) != FX_SUCCESS) goto fail;
    while (len) {
        const ULONG want = len > 512U ? 512U : len;
        ULONG actual = 0U;
        if (fx_file_read(&dl_file, dst, want, &actual) != FX_SUCCESS || actual != want || !flight_sd_io_verified())
            goto fail;
        dst += want; len -= (uint32_t)want;
    }
    return true;
fail:
    ++g_sd_dl_errors;
    dl_close();
    return false;
}

static void dl_hash_step(void)
{
    if (!sd_file_stream_ticket_current(dl_hash.ticket)) { dl_hash.active = false; return; }
    launchcore_sha256_ctx_t *sha = sd_file_stream_scratch();
    uint8_t *const block = sd_file_stream_block();
    uint8_t ans[SD_FILE_HASH_ANSWER] = {SD_FILE_OP_HASH | SD_FILE_RESPONSE_FLAG, SD_FILE_STATUS_OK};
    for (unsigned i = 0; i < SD_DL_HASH_SECTORS_PER_TURN && dl_hash.pos < dl_hash.size; ++i) {
        const uint32_t n = dl_hash.size - dl_hash.pos > 512U ? 512U : dl_hash.size - dl_hash.pos;
        if (!dl_read(NULL, dl_hash.pos, block, n)) {
            ans[1] = SD_FILE_STATUS_IO_ERROR;
            sd_file_stream_answer(dl_hash.ticket, ans, sizeof ans);
            dl_hash.active = false;
            return;
        }
        launchcore_sha256_update(sha, block, n);
        dl_hash.pos += n;
    }
    if (dl_hash.pos < dl_hash.size) { sd_pipeline_wake(); return; }
    ans[2] = dl_hash.growing ? SD_XFER_ANSWER_GROWING : 0U;
    sd_file_put_u32(ans + 3, dl_hash.size);
    launchcore_sha256_final(sha, ans + 7);
    sd_file_stream_answer(dl_hash.ticket, ans, sizeof ans);
    dl_hash.active = false;
}

static void dl_request(const sd_file_request_t *rq, uint32_t ticket)
{
    uint8_t frame[SD_FILE_ANSWER_MAX];
    uint32_t size = 0U;
    uint8_t status = SD_FILE_STATUS_BAD_STATE;
    uint8_t *const block = sd_file_stream_block();
    if (!sd_file_stream_state_allowed() || !block) {
        /* refused below */
    } else if (flight_sd_prepare_io() != FX_SUCCESS) {
        status = SD_FILE_STATUS_IO_ERROR;
    } else if (rq->op == SD_FILE_OP_LIST) {
        uint8_t flags = 0U;
        status = dl_list(rq, (CHAR *)block, &size, &flags);
        const size_t name_len = status == SD_FILE_STATUS_OK ? strlen((const char *)block) : 0U;
        sd_file_stream_answer(ticket, frame, sd_file_encode_list(frame, status, rq->index, size, flags,
                                                                 (const char *)block, name_len));
        if (status == SD_FILE_STATUS_IO_ERROR) ++g_sd_dl_errors;
        return;
    } else if (rq->op == SD_FILE_OP_HASH && sd_file_stream_transfer_active()) {
        status = SD_FILE_STATUS_BUSY;
    } else {
        status = dl_open_file(rq->name, &size);
        if (status == SD_FILE_STATUS_OK && rq->op == SD_FILE_OP_OPEN) {
            sd_file_stream_open(ticket, &rq->open, size, dl_growing(rq->name));
            return;
        }
        if (status == SD_FILE_STATUS_OK) {
            dl_hash.active = true; dl_hash.ticket = ticket; dl_hash.pos = 0U; dl_hash.size = size;
            dl_hash.growing = dl_growing(rq->name);
            launchcore_sha256_init(sd_file_stream_scratch());
            dl_hash_step();
            return;
        }
    }
    if (status == SD_FILE_STATUS_IO_ERROR) ++g_sd_dl_errors;
    size_t n = sd_file_encode_status(frame, rq->op, status);
    if (rq->op == SD_FILE_OP_OPEN) frame[2] = rq->open.xfer;
    sd_file_stream_answer(ticket, frame, n);
}

void sd_file_service_turn(void)
{
    const bool allowed = sd_file_stream_state_allowed();
    const uint32_t now = now_ms();
    const bool busy = dl_hash.active || sd_file_stream_transfer_active();
    if ((!allowed || !sd_file_stream_session_active() ||
         (!busy && (uint32_t)(now - dl_last_ms) >= SD_DL_IDLE_CLOSE_MS)) && (dl_open || dl_list_valid)) {
        dl_close(); dl_list_valid = false;
    }
    if (!allowed || !sd_file_stream_session_active()) { dl_hash.active = false; return; }
    sd_file_request_t rq;
    uint32_t ticket;
    if (!dl_hash.active && sd_file_stream_claim(&rq, &ticket)) { dl_request(&rq, ticket); dl_last_ms = now; }
    if (dl_hash.active) { dl_hash_step(); dl_last_ms = now; return; }
    if (sd_file_stream_transfer_active() && dl_open && sd_file_stream_produce(dl_read, NULL, now)) dl_last_ms = now;
}

/* After sd_conclude (Recovery) or a stop before any log opened: the thread
 * stays alive for reads only. Producers are stopped, so line.full is posted
 * only by sd_pipeline_wake. */
static void sd_file_service_forever(void)
{
    for (;;) {
        if (tx_semaphore_get(&line.full, (TX_TIMER_TICKS_PER_SECOND + 19U) / 20U) == TX_SEMAPHORE_ERROR)
            tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
        sd_file_service_turn();
    }
}
#else
#define sd_file_service_turn() ((void)0)
#define sd_file_service_forever() ((void)0)
#endif

void sd_pipeline_task()
{
    UINT st;
    const char *surprise = "failed:";

    if (sd_pipeline_init(surprise) != WE_ARE_SO_BACK)
    {
        sd_file_service_forever();
        return;
    }

    bool pending_write = false;
    sd_refresh_clock();
    uint32_t refreshed_at = now_ms();
    MrAnalog (load(&g_conf, Acq) & option(SD_Pipeline_Reset))
    {
        const uint32_t clock_now = now_ms();
        if ((uint32_t)(clock_now - refreshed_at) >= 1000U) {
            sd_refresh_clock(); refreshed_at = clock_now;
        }
        /* Poll the tiny IRQ event queue independently of raw-data production.
         * A silent sensor stream must not strand pulse or final-tail records. */
        fc_lock(&line.lock);
        sd_mark_launch_cutoff();
        fc_unlock(&line.lock);
        if (sd_recovery_log_expired()) { sd_conclude(); break; }
        g_fc_sd_state &= (uint8_t)~FC_SD_RETRY_PENDING; /* mounted and open */
        if (flight_sd_prepare_io() != FX_SUCCESS) {
            tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND + 19U) / 20U);
            continue;
        }
        sd_service_events(false);
#if FC_GROUND_TEST
        igt_sd_service(); /* ground-test CSVs: this task owns FileX */
#endif
        const bool flush_due = timer_probe(SDFlush, SD_FLUSH_INTERVAL);
        if (flush_due || sd_launch_pending || sd_cutoff_set || line.off[line.cur] >= SD_POST_MARGIN)
            sd_handoff_tail();
        if (!pending_write && line.free && !sd_cutoff_set)
            (void)tx_semaphore_get(&line.full, (TX_TIMER_TICKS_PER_SECOND + 19U) / 20U);
        if (load(&g_conf, Acq) & option(SD_Pipeline_Reset)) break;
        if ((!line.free || sd_cutoff_set) && !sd_try_write_line())
        {
            pending_write = true;
            /* Retry retained data without waiting for another producer signal. */
            tx_thread_sleep((TX_TIMER_TICKS_PER_SECOND + 19U) / 20U);
            continue;
        }
        pending_write = false;
        if (flush_due)
        {
            st = fx_media_flush(&sdio_disk);
            if (!sd_io_failed(st)) g_sd_flushed_records = g_sd_written_records;
            else log_err(id "flush %s %u", surprise, st);
        }
        sd_file_service_turn();
    }
    sd_pipeline_shutdown(surprise);
    sd_file_service_forever();
}
