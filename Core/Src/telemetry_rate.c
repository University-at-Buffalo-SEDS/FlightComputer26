#include "telemetry_rate.h"

uint32_t fc_telemetry_period_ms(void)
{
  return (1000U + FC_TELEMETRY_RATE_HZ / 2U) / FC_TELEMETRY_RATE_HZ;
}

static uint32_t g_last_emit_ms[29];
static uint32_t g_emitted_types;

bool fc_telemetry_rate_allow(uint32_t data_type, uint32_t now_ms)
{
  const uint32_t id = (uint32_t)data_type;
  switch (id)
  {
    case 101U: /* GPS_DATA */
    case 102U: /* GYRO_DATA */
    case 103U: /* ACCEL_DATA */
    case 104U: /* BATTERY_VOLTAGE */
    case 105U: /* BATTERY_CURRENT */
    case 106U: /* BAROMETER_DATA */
    case 108U: /* FUEL_FLOW */
    case 111U: /* FUEL_TANK_PRESSURE */
    case 118U: /* KG1000 */
    case 119U: /* KG50 */
    case 120U: /* GPS_SATELLITE_NUMBER */
    case 121U: /* EULER_ANGLES */
    case 128U: /* IMU_DATA */
      break;
    default:
      return true;
  }
  const uint32_t index = id - 100U;
  const uint32_t period = fc_telemetry_period_ms();
  if ((g_emitted_types & (1U << index)) != 0U &&
      (uint32_t)(now_ms - g_last_emit_ms[index]) < period)
    return false;
  g_last_emit_ms[index] = now_ms;
  g_emitted_types |= 1U << index;
  return true;
}
