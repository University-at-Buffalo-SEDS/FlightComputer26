#include "board_recovery.h"
#include "fcapi.h"
#include "fc_recovery_owner.h"
/* Pure mapping of the ACTUAL evaluator, not the independently managed cache.
 * No write to sm/global state or deployment GPIO is performed by QUERY. */
static uint8_t evaluator_global_state(void) {
 switch (current()) {
  case Startup: return G_Startup;
  case Postinit: case Armed: return G_Armed;
  case Launch: return G_Launch;
  case Ascent: return G_Ascent;
  case Coast: return G_Coast;
  case Apogee: return G_Apogee;
  case Descent: return G_Descent;
  case Reefing: return G_Reefing;
  case Landed: return G_Landed;
  case Recovery: return G_Recovery;
  default: return G_Aborted;
 }
}
int32_t board_recovery_snapshot(BoardRecoverySnapshot *out)
{
 if (out == NULL) return -10;
 const uint32_t mask = __get_PRIMASK(); __disable_irq();
 *out = (BoardRecoverySnapshot){.actual_state=evaluator_global_state(),
     .flags=BOARD_RECOVERY_STATE_VALID,.abort_epoch=fc_recovery_epoch()};
 if (fc_recovery_control_ready()) {
  out->flags |= BOARD_RECOVERY_SUPPORTED | BOARD_RECOVERY_OUTPUTS_VALID;
  if (deployment_pulse_safe_outputs_locked(&out->actual_outputs_mask))
   out->flags |= BOARD_RECOVERY_SAFE_OUTPUTS;
 }
 if (fc_recovery_abort_latched() || (g_conf & option(In_Aborted_State)))
  out->flags |= BOARD_RECOVERY_ABORT_LATCHED;
 __set_PRIMASK(mask);
 return 0;
}

int32_t board_recovery_apply_safe(uint8_t desired,uint32_t expected,BoardRecoverySnapshot *out)
{
 const int32_t result = desired == 0U ? fc_recovery_request_startup(expected) : -10;
 (void)board_recovery_snapshot(out);
 return result;
}
