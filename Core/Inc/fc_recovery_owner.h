#ifndef FC_RECOVERY_OWNER_H
#define FC_RECOVERY_OWNER_H
#include <stdbool.h>
#include <stdint.h>
/* All shared control state is serialized by the same short single-core IRQ
 * exclusion as deployment_pulse_start. These accessors perform no I/O. */
bool fc_recovery_inhibits_pyro(void);
bool fc_recovery_eval_pause_requested(void);
void fc_recovery_evaluation_quiescent(void);
void fc_recovery_control_service(void);
void fc_recovery_network_service(void);
void fc_recovery_manual_stop(void);
void fc_recovery_manual_stop_original_utc(uint64_t source);
/* Validated transport calls this BEFORE any persistent floor write. */
bool fc_recovery_prepare_startup(uint32_t expected_epoch);
int32_t fc_recovery_request_startup(uint32_t expected_epoch);
uint32_t fc_recovery_epoch(void);
bool fc_recovery_abort_latched(void);
/* Caller holds IRQ exclusion; commanded GPIO readback, not physical deployment. */
void deployment_pulse_cancel_locked(void);
bool deployment_pulse_safe_outputs_locked(uint32_t *mask);
bool fc_recovery_holds_sequence(void);
bool fc_recovery_requires_new_flight(void);
bool fc_recovery_reset_failed(void);
bool fc_recovery_postinit_allowed(uint32_t epoch);
bool fc_recovery_admit_new_flight(bool internal);
/* Current-epoch mission admission, independent of first EKF initialization. */
bool fc_recovery_flight_is_admitted(void);
/* Only successful, semantically validated active-flight checkpoint restore. */
bool fc_recovery_restore_admitted_flight(void);
void fc_recovery_request_eval_stop(void);
/* Fixed implementation hooks in the existing control owner, not callbacks. */
bool fc_recovery_control_ready(void);
bool fc_recovery_control_quiesce(void);
void fc_recovery_control_mark_eval_stopped(void);
bool fc_recovery_control_finalize_startup(void);
bool fc_recovery_control_is_startup(void);
bool fc_recovery_control_new_flight_ready(void);
void fc_recovery_control_new_flight(void);
void fc_recovery_control_wake(void);
bool fc_recovery_control_pending(void);
void fc_recovery_evaluation_running(void);
bool fc_recovery_command_admissible(uint32_t command, bool internal);
bool fc_recovery_request_sensor_restart(void);
bool fc_recovery_take_sensor_restart(void);
void fc_recovery_restore_commissioning_hold(void);
/* RESET-only OTA observation. No authority for flash or generic resources.
 * Call retire only while the existing short IRQ owner is held. */
void fc_recovery_ota_reset_retire_locked(void);
bool fc_recovery_ota_reset_capture(uint32_t *epoch);
bool fc_recovery_ota_reset_commit(uint32_t epoch);
/* Actual ThreadX owner metadata, read-only under final IRQ guard. */
bool fc_recovery_ota_reset_boundary_ready(void);
#endif
