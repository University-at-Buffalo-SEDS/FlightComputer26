/* Runtime monitor lease. Any state can enter; leaving resets the MCU.
 * The distribution task owns the EKF behind the normal recovery/output hold. */
#include "fcapi.h"
#include "fctasks.h"
#include "fc_monitor_bench.h"
#include "fc_recovery_owner.h"
#include <stddef.h>

/* ABI v2 command: bit31 session, bit30 reset estimate, bit29 run EKF,
 * bits0..28 changing lease sequence. Only command is written by the host. */
typedef struct {
  uint32_t magic, command, status, tick, predictions, updates;
  float altitude, velocity, quaternion[4];
} monitor_mailbox;
_Static_assert(sizeof(monitor_mailbox) == 48, "Monitor ABI v2");
_Static_assert(offsetof(monitor_mailbox, command) == 4, "Only writable word");
volatile monitor_mailbox g_fc_monitor = {.magic = 0x46434d32U};

static bool valid_float(float value, float bound)
{
  union { float value; uint32_t bits; } cast = {.value = value};
  return (cast.bits & 0x7f800000U) != 0x7f800000U && value >= -bound && value <= bound;
}

static volatile uint32_t monitor_command;
static volatile bool entered;

/* Runs in the recovery task, including during the normal flight sequence. */
void fc_monitor_bench_poll(void)
{
  static uint32_t renewed;
  const uint32_t now = HAL_GetTick();
  const uint32_t requested = g_fc_monitor.command;
  if (requested != monitor_command) {
    monitor_command = requested; renewed = now;
  }
  if (!(requested & 0x80000000U) || (uint32_t)(now - renewed) > 2000U) {
    if (entered) { NVIC_SystemReset(); return; }
    g_fc_monitor.status = 0; return;
  }
  if (!entered) {
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    entered = fc_recovery_monitor_enter();
    __set_PRIMASK(mask);
  }
  if (!fc_recovery_monitor_ready()) g_fc_monitor.status = 3;
}

/* Only distribution touches the EKF, after recovery has quiesced evaluation. */
void fc_monitor_bench_sample(bool gyro, bool accel, bool baro)
{
  static uint32_t last_command, last_prediction;
  static unsigned fresh;
  static bool initialized, pending_baro;
  const uint32_t command = monitor_command;
  const uint32_t now = HAL_GetTick();
  if (!entered || !fc_recovery_monitor_ready()) return;
  if (command != last_command) {
    last_command = command;
    if (command & 0x40000000U) {
      initialized = false; fresh = 0; pending_baro = false;
      g_fc_monitor.predictions = g_fc_monitor.updates = 0;
    }
  }
  if (!(command & 0x20000000U)) {
    initialized = false; fresh = 0; pending_baro = false;
    g_fc_monitor.status = 0; return;
  }
  if (!valid_float(meas.gyro.x, MAX_DPS) || !valid_float(meas.gyro.y, MAX_DPS) ||
      !valid_float(meas.gyro.z, MAX_DPS) || !valid_float(meas.accl.x, MAX_ACC) ||
      !valid_float(meas.accl.y, MAX_ACC) || !valid_float(meas.accl.z, MAX_ACC) ||
      !valid_float(meas.baro.alt, MAX_ALT)) {
    initialized = false; fresh = 0; g_fc_monitor.status = 1; return;
  }
  fresh |= (gyro ? 1U : 0U) | (accel ? 2U : 0U);
  pending_baro |= baro;
  if (fresh != 3U || (!initialized && !pending_baro)) {
    if (!initialized) g_fc_monitor.status = 1;
    return;
  }
  fresh = 0;
  if (!initialized || (uint32_t)(now - last_prediction) > 100U) {
    if (!kalman_monitor_initialize()) { g_fc_monitor.status = 1; return; }
    initialized = true; last_prediction = now;
  }
  const uint32_t elapsed = now - last_prediction;
  if (!elapsed) return;
  last_prediction = now;
  ascent_predict((float)elapsed / 1000.0f, load(&g_conf, Acq));
  g_fc_monitor.predictions++;
  if (pending_baro) { ascent_update(); pending_baro = false; g_fc_monitor.updates++; }
  else svec(0) = imedsv;
  g_fc_monitor.altitude = svec(0).alt;
  g_fc_monitor.velocity = svec(0).vel;
  g_fc_monitor.quaternion[0] = qv.q0;
  g_fc_monitor.quaternion[1] = qv.rho1;
  g_fc_monitor.quaternion[2] = qv.rho2;
  g_fc_monitor.quaternion[3] = qv.rho3;
  sm.idx = (sm.idx + 1U) & STATE_HISTORY_MASK;
  /* Publish tick after all output fields; readers check both passes. */
  __DMB(); g_fc_monitor.tick = now; g_fc_monitor.status = 2;
}
