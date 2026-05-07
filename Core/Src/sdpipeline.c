/* Core/Src/sdpipeline.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"

#define id "SD "


static FX_FILE file = {0};
static sd_meta line = {.free = true};

uncached char sdbuf[2][SD_BUFFER_SIZE] = {0};


/* Helpers */

static inline fu16 ftoa(char *dst, const float *val, fu8 count)
{
  char *p = dst;
  char reversed[12];

  for (fu8 k = 0; k < count; ++k)
  {
    if (val[k] < 0.0f) *(p++) = '-';

		fi16 idx = 0;
    fu32 scaled = (fu32)(fabsf(val[k]) * F32_SCALE + 0.5f);
    fu32 int_part = scaled / (fu32)F32_SCALE;
    fu32 flt_part = scaled % (fu32)F32_SCALE;
    
    do
		{
      reversed[idx++] = (char)('0' + (int_part % 10u));
      int_part /= 10u;
    }
		while (int_part != 0);

    while (idx > 0) *(p++) = reversed[--idx];

    idx = 0;
    *(p++) = '.';

    for (fu8 i = 0; i < 4; ++i)
		{
      reversed[idx++] = (char)('0' + (flt_part % 10u));
      flt_part /= 10u;
    }
    while (idx > 0) *(p++) = reversed[--idx];

    if (k < count - 1) *(p++) = ' ';
  }

  *p = '\0';
  return (fu16)(p - dst);
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
				tx_semaphore_put(&line.full);
				fc_unlock(&line.lock);
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
	switch (ty) {
		case SEDS_DT_MESSAGE_DATA:		return "Message";
		case SEDS_DT_ORDERED_MESSAGE:	return "Ordered";
		case SEDS_DT_GENERIC_ERROR:		return "Error";
		default:											return "Echo";
	}
}

static inline constexpr char *seds_f32(SedsDataType ty)
{
	switch (ty) {
		case SEDS_DT_IMU_LOCAL:				return "IMU";
		case SEDS_DT_BAROMETER_LOCAL:	return "BARO";
		case SEDS_DT_ACCEL_LOCAL:			return "ACCL";
		case SEDS_DT_GYRO_LOCAL:			return "GYRO";
		case SEDS_DT_ASCENT_LOCAL:    return "EKF";
    case SEDS_DT_DESCENT_LOCAL:   return "DKF";
    case SEDS_DT_EULER_ANGLES:    return "EULER";
		default:											return "QUAT";
	}
}

static inline char *unique_sorted_filename(void)
{
	static char fbuf[26 + sizeof SEDS_LOG_FILENAME];

	tx_thread_sleep(now_ms() % 61);

	fu32 time = now_ms();

	snprintf(fbuf, sizeof fbuf, SEDS_LOG_FILENAME "-%u.log", time);

	return fbuf;
}


/* API */

void sd_append_f32(SedsDataType ty, const float *val, fu8 count)
{
	fu32 relative_ts = now_ms();

	char buf[F32_TO_STR_MAX_SIZE];
	ftoa(buf, val, count);

	fc_lock(&line.lock);

	fu16 rem = SD_BUFFER_SIZE - line.off[line.cur];
	char *off = sdbuf[line.cur] + line.off[line.cur];

	fu16 written = snprintf(off, rem, "%u %s: %s\n",
													relative_ts, seds_f32(ty), buf);

	sd_release_notify(written, rem);
}

void sd_append_string(SedsDataType ty, const char *str)
{
	fu32 relative_ts = now_ms();

	fc_lock(&line.lock);

	fu16 rem = SD_BUFFER_SIZE - line.off[line.cur];
	char *off = sdbuf[line.cur] + line.off[line.cur];

	fu16 written = snprintf(off, rem, "%u %s: %s\n",
													relative_ts, seds_msg(ty), str);

	sd_release_notify(written, rem);
}


/* Task */

static inline bool sd_try_write_line(void)
{
	static bool local = 0;

	fc_lock(&line.lock);
	fu16 write_len = line.off[local];
	fc_unlock(&line.lock);

	sweetbench_start(12);

	UINT st = fx_file_seek(&file,
												 file.fx_file_current_file_size);
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

static inline void sd_pipeline_init(const char *surprise)
{
	fetch_and(&g_conf, ~option(SD_Pipeline_Reset), Rlx);

	UINT st = tx_semaphore_create(&line.full, id "S", 0);

	if (st != TX_SUCCESS)
	{
		log_die(id "sema %s %u", surprise, st);
	}
	do
	{
		file.fx_file_name = unique_sorted_filename();
		st = fx_file_create(&sdio_disk, file.fx_file_name);
	}
	while (st == FX_ALREADY_CREATED);

	if (st != FX_SUCCESS)
	{
		log_die(id "fcreate %s %u", surprise, st);
	}

	st = fx_file_open(&sdio_disk, &file, file.fx_file_name,
																			FX_OPEN_FOR_WRITE);
	if (st != FX_SUCCESS)
	{
    log_die(id "fopen %s %u", surprise, st);
  }
}

void sd_pipeline_task()
{
	UINT st;
	const char *surprise = "failed:";

	sd_pipeline_init(surprise);

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