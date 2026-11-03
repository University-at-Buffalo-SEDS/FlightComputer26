#include "main.h"
#include "fc_recovery_owner.h"
#include "fc_watchdog_recovery.h"
#include "flight_state_cache.h"
#include "persistent_store.h"
#include "board_recovery_floor.h"
#include <stddef.h>

enum { RESET_IDLE, RESET_REQUESTED, RESET_WAIT_EVAL, RESET_NETWORK,
       RESET_COMMITTING, RESET_FINAL, RESET_FAILED };
static volatile struct {
    uint32_t epoch, requested, completed, restart_epoch, commission_epoch;
    uint8_t phase, sensor_restart;
    bool manual_stop, reset_inhibit, new_flight, verified_startup;
    bool stop_handled, legacy_stop, commissioning_hold;
    bool admitted_flight; /* current owner epoch; not the evaluator init flag */
} owner = {.epoch = 1U};
_Static_assert(sizeof owner <= 32U, "Bounded single FC recovery mailbox");

static bool reset_active(void)
{
    return owner.phase >= RESET_REQUESTED && owner.phase <= RESET_FINAL;
}
static void advance_epoch(void)
{
    owner.admitted_flight = false;
    if (owner.epoch != UINT32_MAX) ++owner.epoch;
}
uint32_t fc_recovery_epoch(void) { return owner.epoch; }
bool fc_recovery_reset_failed(void) { return owner.phase == RESET_FAILED; }
bool fc_recovery_holds_sequence(void)
{
    return !persistent_store_controls_ready() || owner.commissioning_hold || owner.manual_stop || owner.reset_inhibit ||
           (!owner.verified_startup && !fc_watchdog_can_actuate());
}
bool fc_recovery_requires_new_flight(void) { return owner.new_flight; }
bool fc_recovery_abort_latched(void)
{
    return owner.commissioning_hold || owner.manual_stop || owner.phase == RESET_FAILED ||
        (!owner.verified_startup && !fc_watchdog_can_actuate());
}
bool fc_recovery_inhibits_pyro(void)
{
    return fc_recovery_holds_sequence() || owner.new_flight;
}
bool fc_recovery_control_pending(void)
{
    return reset_active() || owner.sensor_restart == 1U || owner.legacy_stop ||
           (owner.manual_stop && !owner.stop_handled);
}
bool fc_recovery_eval_pause_requested(void)
{
    return fc_recovery_inhibits_pyro() || owner.legacy_stop || owner.sensor_restart == 1U;
}
bool fc_recovery_postinit_allowed(uint32_t epoch)
{
    return epoch == owner.epoch && !fc_recovery_holds_sequence();
}

void fc_recovery_manual_stop_original_utc(uint64_t source)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if(board_recovery_manual_time_retired(source)){__set_PRIMASK(mask);return;}
    /* FIRST retained store: an IWDG at every subsequent crash point rejects
     * old checkpoint replay. The same guard serializes every pyro HIGH. */
    const bool sealed = fc_watchdog_hold_for_operator_reset();
    /* Only a newly delivered, verified manual marker calls this. Exact SDK
     * duplicates are retired by SDK ownership; every new Stop invalidates
     * an older readback ticket even when already latched. */
    advance_epoch();
    if (reset_active() || !sealed) owner.phase = RESET_FAILED;
    owner.manual_stop = true;
    owner.reset_inhibit = true;
    owner.new_flight = true;
    owner.verified_startup = false;
    owner.stop_handled = false;
    owner.sensor_restart = 0U;
    deployment_pulse_cancel_locked();
    __set_PRIMASK(mask);
    fc_recovery_control_wake();
}

/* Admission proof is sampled under the same IRQ owner as Stop/pyro. Sensor
 * restart itself does not revoke the already-admitted mission. */
bool fc_recovery_flight_is_admitted(void)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    const bool valid = owner.admitted_flight && !owner.new_flight &&
        !owner.legacy_stop && owner.phase != RESET_FAILED &&
        !fc_recovery_holds_sequence();
    __set_PRIMASK(mask);
    return valid;
}
/* Boot-only call site: a complete active-flight checkpoint was already
 * semantically validated. Never infer this from a generic resumed flag. */
bool fc_recovery_restore_admitted_flight(void)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    const bool valid = !owner.new_flight && !owner.legacy_stop &&
        !reset_active() && owner.phase != RESET_FAILED &&
        !fc_recovery_holds_sequence();
    if (valid) owner.admitted_flight = true;
    __set_PRIMASK(mask);
    return valid;
}

bool fc_recovery_request_sensor_restart(void)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (fc_recovery_inhibits_pyro()) { __set_PRIMASK(mask); return false; }
    if (owner.sensor_restart == 0U) {
        owner.restart_epoch = owner.epoch;
        owner.sensor_restart = 1U;
    }
    __set_PRIMASK(mask);
    fc_recovery_control_wake();
    return true;
}
bool fc_recovery_take_sensor_restart(void)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    const bool ready = owner.sensor_restart == 2U;
    const bool valid = ready && owner.restart_epoch == owner.epoch &&
                       !fc_recovery_inhibits_pyro();
    if (ready) owner.sensor_restart = 0U;
    __set_PRIMASK(mask);
    return valid;
}

void fc_recovery_request_eval_stop(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    owner.admitted_flight = false;
    if (reset_active() || owner.sensor_restart != 0U) {
        advance_epoch();
        if (reset_active()) owner.phase = RESET_FAILED;
        /* Do not release an already acquired reset/pyro inhibit. */
    }
    owner.sensor_restart = 0U;
    owner.legacy_stop = true;
    __set_PRIMASK(mask);
    fc_recovery_control_wake();
}

/* Transport has already validated identity, boot, epoch and canonical time.
 * Seal readback precedes even the replay-floor write. A failed barrier cannot
 * start asynchronous persistence; runtime LOW/inhibit remains conservative. */
bool fc_recovery_prepare_startup(uint32_t expected)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (!fc_recovery_control_ready() || expected == 0U ||
        expected == UINT32_MAX || expected != owner.epoch || reset_active()) {
        __set_PRIMASK(mask); return false;
    }
    owner.admitted_flight = false;
    const bool retained = fc_watchdog_hold_for_operator_reset();
    deployment_pulse_cancel_locked();
    if (!retained) {
        owner.reset_inhibit = true;
        owner.new_flight = true;
        owner.verified_startup = false;
        owner.phase = RESET_FAILED;
    }
    if (retained && owner.phase == RESET_FAILED)
        owner.phase = RESET_IDLE; /* Only a fresh transport intent may retry. */
    __set_PRIMASK(mask);
    return retained;
}

int32_t fc_recovery_request_startup(uint32_t expected)
{
    if (!fc_recovery_control_ready()) return -14;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (expected != 0U && expected != UINT32_MAX &&
        owner.completed == expected && owner.epoch == expected + 1U &&
        !owner.manual_stop && fc_recovery_control_is_startup()) {
        uint32_t outputs;
        const bool safe = deployment_pulse_safe_outputs_locked(&outputs);
        __set_PRIMASK(mask);
        return safe ? 0 : -9;
    }
    if (expected == 0U || expected == UINT32_MAX || expected != owner.epoch) {
        __set_PRIMASK(mask); return -10;
    }
    if (reset_active()) { __set_PRIMASK(mask); return -14; }
    if (owner.phase == RESET_FAILED) { __set_PRIMASK(mask); return -9; }
    owner.admitted_flight = false;
    if (!fc_watchdog_hold_for_operator_reset()) {
        owner.reset_inhibit = true;
        owner.new_flight = true;
        owner.verified_startup = false;
        owner.phase = RESET_FAILED;
        deployment_pulse_cancel_locked();
        __set_PRIMASK(mask); return -9;
    }
    owner.requested = expected;
    owner.sensor_restart = 0U;
    owner.phase = RESET_REQUESTED;
    owner.reset_inhibit = true;
    owner.new_flight = true;
    owner.verified_startup = false;
    __set_PRIMASK(mask);
    fc_recovery_control_wake();
    return -14;
}

/* Only the existing priority-0 control owner calls this. It performs no SDK,
 * flash, logging, or sensor initialization and never spins awaiting a peer. */
void fc_recovery_control_service(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Restores the prior explicit durable commissioning Stop after cold boot.
     * RAM inhibit was established before watchdog replay; seal only now, after
     * watchdog initialization. No flash/SDK or evaluator work under IRQ here. */
    if(owner.commissioning_hold&&owner.manual_stop&&!owner.stop_handled){
        deployment_pulse_cancel_locked();
        if(!fc_watchdog_hold_for_operator_reset())owner.phase=RESET_FAILED;
    }
    if (owner.phase == RESET_REQUESTED) {
        deployment_pulse_cancel_locked();
        owner.phase = RESET_WAIT_EVAL;
    }
    const bool need_stop = owner.sensor_restart == 1U || owner.legacy_stop ||
        (owner.manual_stop && !owner.stop_handled) || owner.phase == RESET_WAIT_EVAL;
    __set_PRIMASK(mask);
    if (need_stop && fc_recovery_control_quiesce()) {
        fc_recovery_control_mark_eval_stopped();
        mask = __get_PRIMASK(); __disable_irq();
        owner.legacy_stop = false;
        if (owner.sensor_restart == 1U) {
            owner.sensor_restart = owner.restart_epoch == owner.epoch &&
                                   !fc_recovery_inhibits_pyro() ? 2U : 0U;
        }
        if (owner.manual_stop) owner.stop_handled = true;
        if (owner.phase == RESET_WAIT_EVAL && owner.requested == owner.epoch)
            owner.phase = RESET_NETWORK;
        __set_PRIMASK(mask);
    }
    mask = __get_PRIMASK(); __disable_irq();
    if (owner.phase == RESET_FINAL && owner.requested == owner.epoch) {
        uint32_t outputs;
        const uint32_t completing = owner.epoch;
        const bool finalized = deployment_pulse_safe_outputs_locked(&outputs) &&
                               fc_recovery_control_finalize_startup();
        if (owner.phase != RESET_FINAL || owner.epoch != completing) {
            /* A cancellation must never be overwritten by completion. */
        } else if (!finalized) {
            owner.phase = RESET_FAILED;
        } else {
            owner.completed = owner.requested;
            advance_epoch();
            owner.phase = RESET_IDLE;
            owner.manual_stop = false;
            owner.reset_inhibit = false;
            owner.verified_startup = true;
            owner.stop_handled = false;
            /* new_flight/retained Stop seal remain: Startup does NOT rearm. */
        }
    }
    __set_PRIMASK(mask);
}

/* Only the serialized telemetry owner calls this, outside a packet callback.
 * It never waits for control. Cancellation during I/O cannot finalize reset. */
void fc_recovery_network_service(void)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (owner.phase != RESET_NETWORK || owner.requested != owner.epoch) {
        __set_PRIMASK(mask); return;
    }
    const uint32_t epoch = owner.epoch;
    owner.phase = RESET_COMMITTING;
    __set_PRIMASK(mask);
    const int32_t result = flight_state_cache_reset_startup();
    mask = __get_PRIMASK(); __disable_irq();
    if (owner.phase == RESET_COMMITTING && owner.epoch == epoch)
        owner.phase = result == 0 ? RESET_FINAL : RESET_FAILED;
    __set_PRIMASK(mask);
    fc_recovery_control_wake();
}

bool fc_recovery_admit_new_flight(bool internal)
{
    const uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (!owner.new_flight) {
        const bool allowed = !fc_recovery_holds_sequence() && fc_watchdog_can_actuate();
        if (allowed && !internal) owner.admitted_flight = true;
        __set_PRIMASK(mask); return allowed;
    }
    if (internal || !owner.verified_startup || fc_recovery_holds_sequence() ||
        owner.epoch == UINT32_MAX || !fc_recovery_control_new_flight_ready() ||
        !fc_watchdog_begin_new_flight()) {
        __set_PRIMASK(mask); return false;
    }
    fc_recovery_control_new_flight();
    advance_epoch(); /* Old recovery tickets cannot cross a new flight. */
    owner.verified_startup = false;
    owner.new_flight = false;
    owner.admitted_flight = true; /* only the checked explicit branch reaches here */
    __set_PRIMASK(mask);
    return true;
}

/* Boot caller only, after GPIO init and before watchdog replay/control tasks.
 * This is restoration of an explicit durable BRPC PENDING, never an Abort
 * caused merely by virgin/unknown media. No retained-RAM persistence claim. */
void fc_recovery_restore_commissioning_hold(void){
 const uint32_t mask=__get_PRIMASK();__disable_irq();
 owner.admitted_flight=false;
 owner.commissioning_hold=true;owner.commission_epoch=owner.epoch;
 owner.manual_stop=true;owner.reset_inhibit=true;owner.new_flight=true;
 owner.verified_startup=false;owner.stop_handled=false;owner.sensor_restart=0;
 deployment_pulse_cancel_locked();__set_PRIMASK(mask);
}
int32_t board_recovery_commission_prepare(uint32_t expected){
 if(!fc_recovery_prepare_startup(expected))return expected==owner.epoch?-9:-10;
 const uint32_t mask=__get_PRIMASK();__disable_irq();
 if(expected!=owner.epoch||reset_active()){__set_PRIMASK(mask);return -10;}
 if(!owner.commissioning_hold||owner.commission_epoch!=expected){
  owner.commission_epoch=expected;owner.stop_handled=false;
 }
 owner.commissioning_hold=true;owner.manual_stop=true;owner.reset_inhibit=true;
 owner.new_flight=true;owner.verified_startup=false;owner.sensor_restart=0;
 deployment_pulse_cancel_locked();__set_PRIMASK(mask);fc_recovery_control_wake();return 0;
}
bool board_recovery_commission_held(uint32_t expected){
 const uint32_t mask=__get_PRIMASK();__disable_irq();uint32_t outputs;
 bool valid=expected==owner.epoch&&owner.commissioning_hold&&owner.manual_stop&&owner.stop_handled&&owner.phase!=RESET_FAILED&&fc_recovery_control_ready()&&deployment_pulse_safe_outputs_locked(&outputs);
 __set_PRIMASK(mask);return valid;
}
int32_t board_recovery_commission_apply(uint32_t expected){return fc_recovery_request_startup(expected);}
bool board_recovery_commission_applied(uint32_t expected,uint32_t*actual){
 const uint32_t mask=__get_PRIMASK();__disable_irq();uint32_t outputs;
 bool valid=actual&&expected&&expected!=UINT32_MAX&&owner.commissioning_hold&&owner.completed==expected&&owner.epoch==expected+1U&&!owner.manual_stop&&fc_recovery_control_is_startup()&&deployment_pulse_safe_outputs_locked(&outputs);
 if(valid){*actual=owner.epoch;}
 __set_PRIMASK(mask);return valid;
}
bool board_recovery_commission_finish(uint32_t expected,uint32_t actual){
 const uint32_t mask=__get_PRIMASK();__disable_irq();uint32_t live=0;
 bool valid=board_recovery_commission_applied(expected,&live)&&live==actual&&persistent_store_publish_commission_complete();
 if(valid){owner.commissioning_hold=false;}
 __set_PRIMASK(mask);return valid;
}
void board_recovery_commission_retain(void){
 const uint32_t mask=__get_PRIMASK();__disable_irq();
 owner.admitted_flight=false;
 owner.commissioning_hold=true;owner.manual_stop=true;owner.reset_inhibit=true;
 owner.new_flight=true;owner.verified_startup=false;owner.stop_handled=false;
 owner.sensor_restart=0;owner.phase=RESET_FAILED;deployment_pulse_cancel_locked();
 __set_PRIMASK(mask);fc_recovery_control_wake();
}

void fc_recovery_manual_stop(void){fc_recovery_manual_stop_original_utc(0U);}
