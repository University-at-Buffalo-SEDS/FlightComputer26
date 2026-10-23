#include "telemetry_rate.h"
#include "sedsnet_config.h"

uint32_t fc_telemetry_period_ms(void)
{
  return FC_TELEMETRY_PERIOD_MS;
}

/* Runtime schema IDs are stable names, not a contiguous v3 enum range. Keep
 * one bounded slot per sampled stream, independent of its numeric identity. */
static const uint32_t g_rate_limited_types[] = {
  SEDS_DT_GPS_DATA, SEDS_DT_GYRO_DATA, SEDS_DT_ACCEL_DATA,
  SEDS_DT_BATTERY_VOLTAGE, SEDS_DT_BATTERY_CURRENT, SEDS_DT_BAROMETER_DATA,
  SEDS_DT_FUEL_FLOW, SEDS_DT_FUEL_TANK_PRESSURE, SEDS_DT_KG1000,
  SEDS_DT_KG50, SEDS_DT_GPS_SATELLITE_NUMBER, SEDS_DT_EULER_ANGLES,
  SEDS_DT_IMU_DATA,
};
#define RATE_STREAM_COUNT (sizeof(g_rate_limited_types) / sizeof(g_rate_limited_types[0]))
_Static_assert(RATE_STREAM_COUNT <= 32U, "rate stream mask must fit");
static uint32_t g_last_emit_ms[RATE_STREAM_COUNT];
static uint32_t g_emitted_types;

bool fc_telemetry_rate_allow(uint32_t data_type, uint32_t now_ms)
{
  for (uint32_t index = 0; index < RATE_STREAM_COUNT; ++index)
  {
    if (g_rate_limited_types[index] != data_type)
      continue;
    const uint32_t period = fc_telemetry_period_ms();
    if ((g_emitted_types & (1U << index)) != 0U &&
        (uint32_t)(now_ms - g_last_emit_ms[index]) < period)
      return false;
    g_last_emit_ms[index] = now_ms;
    g_emitted_types |= 1U << index;
    return true;
  }
  return true;
}
