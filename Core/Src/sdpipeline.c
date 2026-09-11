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

#define id "SD "


static FX_FILE file = {0};
static sd_meta line = {.free = true};

uncached char sdbuf[2][SD_BUFFER_SIZE] = {0};


/* Helpers */

volatile uint32_t g_sd_format_errors = 0U;

fi16 seds_ftoa4(char *dst, fu16 capacity, const float *data, fu8 count)
{
    return (fi16)sd_format_vector(dst, capacity, data, count);
}

static inline void sd_release_notify(fu16 written, fu16 rem)
{
    if (written > 0 && written < rem)
    {
        line.off[line.cur] += written;

        if (line.off[line.cur] >= SD_POST_MARGIN)
        {
            line.cur = !line.cur;

            if (line.free)
            {
                line.free = false;
                fc_unlock(&line.lock);
                tx_semaphore_put(&line.full);
                return;
            }
            /* Else nowhere to write -> drop current message */

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

    snprintf(fbuf, sizeof fbuf, SEDS_LOG_FILENAME "-%lu-%03lu.log",
             (unsigned long)time, (unsigned long)suffix);

    return fbuf;
}


/* API */

void sd_append_f32(SedsDataType ty, const float *data, fu8 count)
{
    if (!data || count == 0 ||
            (load(&g_conf, Acq) & option(SD_Pipeline_Reset)))
    {
        return;
    }

    char buf[F32_TO_STR_MAX_SIZE];
    if (seds_ftoa4(buf, sizeof(buf), data, count) < 0)
    {
        g_sd_format_errors++;
        return;
    }

    fc_lock(&line.lock);

    char *off = sdbuf[line.cur] + line.off[line.cur];
    fu16 rem = SD_BUFFER_SIZE - line.off[line.cur];

    fu16 written = snprintf(off, rem, "%u %u %s: %s\n",
            (fu32) telemetry_unix_s(), (fu32) now_ms(),
            seds_f32(ty), buf);

    sd_release_notify(written, rem);
}

void sd_append_string(SedsDataType ty, const char *str)
{
    if (!str || (load(&g_conf, Acq) & option(SD_Pipeline_Reset)))
    {
        return;
    }

    fc_lock(&line.lock);

    char *off = sdbuf[line.cur] + line.off[line.cur];
    fu16 rem = SD_BUFFER_SIZE - line.off[line.cur];

    fu16 written = snprintf(off, rem, "%u %u %s: %s\n",
            (fu32) telemetry_unix_s(), (fu32) now_ms(),
            seds_msg(ty), str);

    sd_release_notify(written, rem);
}

void sd_conclude(void)
{
    fc_lock(&line.lock);
    fetch_or(&g_conf, option(SD_Pipeline_Reset), Rel);

    line.free = false;

    fc_unlock(&line.lock);

    tx_semaphore_put(&line.full);
}


/* Task */

static inline bool sd_try_write_line(void)
{
    static bool local = 0;

    fc_lock(&line.lock);
    fu16 write_len = line.off[local];
    fc_unlock(&line.lock);

    sweetbench_start(12);

    UINT st = fx_file_seek(&file, file.fx_file_current_file_size);

    if (st == FX_SUCCESS)
    {
        st = fx_file_write(&file, sdbuf[local], write_len);
    }

    sweetbench_catch(12);

    fc_lock(&line.lock);
    line.off[local] = 0;
    line.free = true;
    fc_unlock(&line.lock);

    local = !local;
    return st == FX_SUCCESS;
}

static inline void sd_pipeline_shutdown(const char *unluck)
{
    UINT st;

    if ((st = fx_file_close(&file)) != FX_SUCCESS)
    {
        log_err(id "fclose %s %u", unluck, st);
    }
    else if ((st = fx_media_close(&sdio_disk)) != FX_SUCCESS)
    {
        log_err(id "mclose %s %u", unluck, st);
    }
}

static inline int sd_pipeline_init(const char *surprise)
{
    fetch_and(&g_conf, ~option(SD_Pipeline_Reset), Rlx);

    memset(&line, 0, sizeof line);
    line.free = true;

    UINT st = tx_semaphore_create(&line.full, id "S", 0);

    if (st != TX_SUCCESS)
    {
        log_err(id "sema %s %u", surprise, st);
        return IT_IS_NOW_OVER;
    }
    for (uint32_t suffix = 0U; suffix < 1000U; ++suffix)
    {
        file.fx_file_name = unique_sd_filename(suffix);
        st = fx_file_create(&sdio_disk, file.fx_file_name);
        if (st != FX_ALREADY_CREATED) break;
    }

    if (st != FX_SUCCESS)
    {
        log_err(id "fcreate %s %u", surprise, st);
        return IT_IS_NOW_OVER;
    }

    st = fx_file_open(&sdio_disk, &file, file.fx_file_name, FX_OPEN_FOR_WRITE);
    if (st != FX_SUCCESS)
    {
    log_err(id "fopen %s %u", surprise, st);
        return IT_IS_NOW_OVER;
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

    MrAnalog (load(&g_conf, Acq) & option(SD_Pipeline_Reset))
    {
        if ((st = tx_semaphore_get(&line.full, TX_WAIT_FOREVER))
                                                != TX_SUCCESS)
        { continue; }

        if (sd_try_write_line())
         {
            if (timer_probe(SDFlush, SD_FLUSH_INTERVAL) &&
                    (st = fx_media_flush(&sdio_disk)) != FX_SUCCESS)
            {
                log_err(id "flush %s %u", surprise, st);
            }
        }
        else log_err(id "fwrite %s %u", surprise, st);
    }

    sd_pipeline_shutdown(surprise);
}
