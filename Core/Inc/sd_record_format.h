#ifndef SD_RECORD_FORMAT_H
#define SD_RECORD_FORMAT_H
#include "sd_float_format.h"
#include "board_uptime.h"
#include <stddef.h>
#include <stdio.h>
#define SD_CSV_HEADER "timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value,time_source,adc_temperature_c,adc_temperature_code,uptime_ticks,uptime_frequency_hz,record_sequence\r\n"
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
