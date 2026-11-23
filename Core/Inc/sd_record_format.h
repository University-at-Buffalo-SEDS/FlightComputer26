#ifndef SD_RECORD_FORMAT_H
#define SD_RECORD_FORMAT_H
#include "sd_float_format.h"
#include "board_uptime.h"
#include <stddef.h>
#include <stdio.h>
#define SD_CSV_HEADER "timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value,time_source,adc_temperature_c,adc_temperature_code,uptime_ticks,uptime_frequency_hz,record_sequence\r\n"
/* SEDS-SD format block (lines 2+), written after SD_CSV_HEADER in every file
 * and part. The schema hash is SHA-256 (first 8 hex) over the columns line and
 * every records line; tests/test_sd_format_block.py recomputes it. Changing a
 * label, column, unit or tick rate here, or the row layout, requires bumping
 * SD_FORMAT_VERSION, freezing the new hash in that test and registering the
 * version in the GroundStation26 SD parser. Pure literals: no formatting code. */
#ifndef FC_FIRMWARE_REV
#define FC_FIRMWARE_REV "unknown" /* CMake: git rev-parse --short=12 HEAD */
#endif
#define SD_FORMAT_VERSION "2"
#define SD_FORMAT_SCHEMA "e59ae8a7"
#define SD_FORMAT_COLUMNS "# seds-sd-columns,u64:ms,u64:ms,label,vec,vec,vec,vec,clock,f32:degC,i32:code,u64:tick,u32:Hz,u64:seq\r\n"
#define SD_FORMAT_RECORDS \
    "# seds-sd-records,accelerometer_sample:v=m/s2*3:a=i16*3:r=i16*3:c=m/s2*3,gyroscope_sample:v=deg/s*3:a=i16*3:r=i16*3:c=deg/s*3,barometer_sample:v=Pa;degC;m:a=u24*2;nan:r=u24*2;nan:c=Pa;degC;m\r\n" \
    "# seds-sd-records,IMU:v=m/s2*3;deg/s*3,BARO:v=m;Pa;degC,EKF:v=1*4;m;m/s,DKF:v=deg*2;m;m/s,EULER:v=deg*3,QUAT:v=1*6,SIM:v=1;s;1*6,ACCL:v=m/s2*3,GYRO:v=deg/s*3\r\n" \
    "# seds-sd-records,Message:v=text,Ordered:v=text,Error:v=text,Echo:v=text,launch_*:v=text,pyro_*:v=text\r\n"
/* tick_hz below is BOARD_UPTIME_FREQUENCY_HZ (asserted in sdpipeline.c). */
#define SD_FORMAT_BLOCK "# seds-sd,format=fc,version=" SD_FORMAT_VERSION ",board=FC,firmware=" \
    FC_FIRMWARE_REV ",tick_hz=1000,schema=" SD_FORMAT_SCHEMA "\r\n" SD_FORMAT_COLUMNS SD_FORMAT_RECORDS
#define SD_FILE_HEADER SD_CSV_HEADER SD_FORMAT_BLOCK
/* Sensor names are internal schema labels, not caller-controlled CSV text.
 * All message/value text is quoted with RFC 4180 escaping. */
/* Both record kinds share the formatter so additional sensor columns do not
 * duplicate clock conversion, escaping and bounds checks in flash. */
#if defined(__GNUC__)
__attribute__((noinline))
#endif
static int sd_format_csv_common(char *dst, size_t capacity, uint64_t utc_ms,
    uint64_t uptime, const char *sensor, const char *value, const char *raw,
    uint64_t sequence)
{
    if (!dst || !capacity || !sensor || !value) return -1;
    char stamp[21], ticks[21], seq[21];
    sd_format_u64(stamp, utc_ms ? utc_ms : uptime);
    sd_format_u64(ticks, uptime); sd_format_u64(seq, sequence);
    int length = snprintf(dst, capacity, "%s,%s,%s,\"", stamp, ticks, sensor);
    if (length < 0 || (size_t)length >= capacity) { dst[0] = 0; return -1; }
    size_t used = (size_t)length;
    for (const char *p = value; *p; ++p) {
        size_t need = *p == '\"' ? 2U : 1U;
        if (need >= capacity - used) { dst[0] = 0; return -1; }
        dst[used++] = *p; if (*p == '\"') dst[used++] = '\"';
    }
    /* Raw is a formatter-generated numeric vector, never caller CSV text.
     * Empty optional columns remain CSV empty strings for ordinary records. */
    length = snprintf(dst + used, capacity - used,
        "\",\"%s\",\"%s\",\"%s\",%s,,,%s,%u,%s\r\n",
        raw ? raw : "", raw ? raw : "", raw ? value : "",
        utc_ms ? "network" : "local", ticks, BOARD_UPTIME_FREQUENCY_HZ, seq);
    if (length < 0 || (size_t)length >= capacity - used) { dst[0] = 0; return -1; }
    return (int)(used + (size_t)length);
}
static inline int sd_format_csv_record(char *dst, size_t capacity, uint64_t utc_ms,
    uint64_t uptime, const char *sensor, const char *value, uint64_t sequence)
{
    return sd_format_csv_common(dst,capacity,utc_ms,uptime,sensor,value,NULL,sequence);
}
static inline int sd_format_csv_sensor_record(char *dst, size_t capacity,
    uint64_t utc_ms, uint64_t uptime, const char *sensor,
    const char *raw, const char *converted, uint64_t sequence)
{
    if (!raw) return -1;
    return sd_format_csv_common(dst,capacity,utc_ms,uptime,sensor,converted,raw,sequence);
}

#endif
