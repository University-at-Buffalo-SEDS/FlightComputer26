/* Core/Src/control_thread.c */

#include "platform.h"
#include "board_watchdog.h"
#include "fc_watchdog_recovery.h"
#include "fc_recovery_owner.h"
#include "fc_monitor_bench.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "FC-Threads.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"
#include "fc_estimator_status.h"
#include "ground_test.h"
#ifdef TELEMETRY_ENABLED
#include "av_bay_underglow.h"
#endif
#include "tx_thread.h"
#include "thread_comm.h"

#define id "RE "


TX_THREAD control_thread;

volatile uint32_t g_network_flight_commands_processed
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_recovery_stack_used
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_recovery_stack_remaining
    __attribute__((used, externally_visible)) = CONTROL_STACK_BYTES;
volatile uint32_t g_recovery_commands_dequeued
    __attribute__((used, externally_visible)) = 0U;
volatile uint32_t g_recovery_receive_loop_ready
    __attribute__((used, externally_visible)) = 0U;

TX_BYTE_POOL *tx_app_shared;

static void recovery_update_stack_profile(void)
{
  _tx_thread_stack_analyze(&control_thread);
  const uintptr_t start = (uintptr_t)control_thread.tx_thread_stack_start;
  const uintptr_t end = (uintptr_t)control_thread.tx_thread_stack_end;
  const uintptr_t highest =
      (uintptr_t)control_thread.tx_thread_stack_highest_ptr;
  if (highest >= start && highest <= end)
  {
    g_recovery_stack_used = (uint32_t)(end - highest + sizeof(ULONG));
    g_recovery_stack_remaining = (uint32_t)(highest - start);
  }
}

volatile fu32 local_time[Time_Users] = {0};

volatile log_rates rates = {LOG_RATE_SD, LOG_RATE_GND};

const led_gpio light[Leds] = {
  [Green] = {LED1_PORT, LED1_PIN},
  [Blue]  = {LED2_PORT, LED2_PIN},
};

atomic_uint_fast32_t g_conf = FC_DEFAULTS | USER_OPTIONS;

static sysmon smon = {TO_ABORT, TO_REINIT, 0, 0, 0};

atomic_uint_fast16_t devctr_if[MEMS_Devices] = {0};

static struct baro_config baro_conf = {
    .osr_t = Baro_OSR_x1,
    .osr_p = Baro_OSR_x8,
    .odr = Baro_ODR_25,
    .iir_coef = Baro_IIR_Coef_3,
    .rezero = 1,
};


#if BOARD_WATCHDOG_ENABLE
const fc_resume_region fc_resume_recovery[] = {
  FC_RESUME_REGION(g_conf),
  FC_RESUME_REGION(local_time),
  { NULL, 0U }
};
#endif

static struct gyro_config gyro_conf = {
    .rng = Gyro_Range_2000Dps,
    .bw = Gyro_32Hz_ODR_100Hz,
};

static struct accl_config accl_conf = {
    .mode = Normal_100Hz,
    .rng = Accl_Range_24g,
};

/* Sensor init */

static inline void sensor_init_supervised(sens_init sn)
{
  sens_init fails = 0;

  sweetbench_start(5, 1);

  clear_spi1_irq();

  if (sn & Baro_Mask)
  {
    try_init_sensor(baro_init(&hspi1, &baro_conf),
                                fails, Baro_Mask);
  }

  if (sn & Gyro_Mask)
  {
    try_init_sensor(gyro_init(&hspi1, &gyro_conf),
                                fails, Gyro_Mask);
  }

  if (sn & Accl_Mask)
  {
    try_init_sensor(accl_init(&hspi1, &accl_conf),
                                fails, Accl_Mask);
  }

  restore_spi1_irq();

  if (sn & Shut_Baro)
  {
    irq_off(Baro_EXTI);
  }
  else
  {
    irq_on(Baro_EXTI);
  }

  if (sn & Shut_Gyro)
  {
    irq_off(Gyro_EXTI_1);
/*  irq_off(Gyro_EXTI_2);   not used for IREC 2026 */
  }
  else
  {
    irq_on(Gyro_EXTI_1);
/*  irq_on(Gyro_EXTI_2);   not used for IREC 2026 */
  }

  if (sn & Shut_Accl)
  {
    irq_off(Accl_EXTI_1);
/*  irq_off(Accl_EXTI_2);   not used for IREC 2026 */
  }
  else
  {
    irq_on(Accl_EXTI_1);
/*  irq_on(Accl_EXTI_2);   not used for IREC 2026 */
  }
  
  if (fails != 0)
  {
    g_conf |= option(Init_Failure_Record);
    log_metric(id "init failures", fails, true);
  }

  sweetbench_catch(5);
}


/* Cooperative evaluator/reset owner. Never terminate a task at an arbitrary
 * lock/Kalman point. No SDK/NVM/logging occurs in these owner hooks. */
static volatile uint8_t evaluation_quiescent = 1U;
bool fc_recovery_control_ready(void) { return g_recovery_receive_loop_ready != 0U; }
bool fc_recovery_control_is_startup(void) { return current() == Startup; }
/* Same short IRQ owner as Launch/Stop/monitor admission; no IO or providers.
 * A lost ground station does not refuse: the bench lease needs a wired probe,
 * and Startup/Armed without a Launch request or admitted flight is on the pad. */
bool fc_recovery_control_monitor_preflight(void)
{
  return (current() == Startup || current() == Armed) &&
      !(load(&g_conf, Acq) & option(Launch_Requested));
}
void fc_recovery_evaluation_running(void) { evaluation_quiescent = 0U; }
void fc_recovery_evaluation_quiescent(void)
{
  evaluation_quiescent = 1U;
  __DMB();
  (void)tx_thread_suspend(&estimator_thread);
}
void fc_recovery_control_wake(void)
{
  const uint32_t mask = __get_PRIMASK(); __disable_irq();
  /* Do not abort a sensor-init sleep or any unrelated wait. */
  if (g_recovery_receive_loop_ready &&
      thread_comm_receiver_waiting(&control_thread))
    (void)tx_thread_wait_abort(&control_thread);
  __set_PRIMASK(mask);
}
bool fc_recovery_control_quiesce(void)
{
  const UINT state = estimator_thread.tx_thread_state;
  if (state == TX_SUSPENDED && !evaluation_quiescent) {
    /* A previously reset thread enters its barrier before flight-mode setup. */
    (void)tx_thread_resume(&estimator_thread);
    return false;
  }
  if (state != TX_COMPLETED && state != TX_TERMINATED &&
      !(state == TX_SUSPENDED && evaluation_quiescent)) {
    (void)tx_event_flags_set(&eval_stage, Baro_Mask | Gyro_Mask | Accl_Mask, TX_OR);
    return false;
  }
  if (load(&meas_locks[IMU].lock, Acq) || load(&meas_locks[Baro].lock, Acq))
    return false;
  if (state != TX_TERMINATED && tx_thread_terminate(&estimator_thread) != TX_SUCCESS)
    return false;
  if (tx_thread_reset(&estimator_thread) != TX_SUCCESS) return false;
  evaluation_quiescent = 1U;
  if (fc_recovery_holds_sequence()) {
    thread_comm_flush();
  }
  return true;
}
void fc_recovery_control_mark_eval_stopped(void)
{
  g_conf |= option(Eval_Abort_Flag);
  /* A preflight monitor lease also stops an evaluator that never ran. */
  if (g_fc_est_phase != FC_EST_IDLE) g_fc_est_phase = FC_EST_STOPPED;
}
/* The reset to Startup's evaluation cancel (replaces the operator's
 * Evaluation Abort): the estimator's flight memory and the sensor-failure
 * monitor return to their boot values. Memory only; IRQs are masked. */
static void fc_cancel_evaluation(void)
{
  estimator_cancel_evaluation();
  smon.to_abort = TO_ABORT;
  smon.to_reinit = TO_REINIT;
  smon.failures = smon.triggers = 0;
  smon.gps_delayed = smon.gps_malform = 0;
  devctr_if[IMU] = devctr_if[Baro] = 0;
}
bool fc_recovery_control_finalize_startup(void)
{
  if (!evaluation_quiescent || estimator_thread.tx_thread_state != TX_SUSPENDED ||
      !fc_watchdog_startup_reset_ready()) return false;
  /* Preserve calibration/biases, samples, clocks, rates, recording shutdown
   * and historical deployment bits until a NEW flight. The evaluation itself
   * is cancelled (what the retired operator Evaluation Abort did): the
   * estimator's flight memory and the sensor-failure counters and thresholds
   * return to their boot values, so nothing from a previous evaluation
   * survives the reset. */
  g_conf &= ~option(Postinit_Requested | Launch_Requested | In_Aborted_State |
                    Graceful_Reset | Defer_Baro_Fallback | Eval_Abort_Flag);
  fc_cancel_evaluation();
  g_conf |= option(Using_Ascent_KF);
  fc_recovery_ota_reset_retire_locked();
  sm.flight = Startup;
  sm.global_state = G_Startup;
  sm.confidence = 0;
  sm.kf_deviations = 0;
  baro_conf.rezero = 0;
  g_fc_est_phase = FC_EST_IDLE;
  g_fc_est_cycles = g_fc_est_start = 0;
#if FC_GROUND_TEST
  igt_stop();
#endif
  return true;
}
bool fc_recovery_control_new_flight_ready(void)
{
  return current() == Armed && !(g_conf & option(Postinit_Requested));
}
void fc_recovery_control_new_flight(void)
{
  g_conf &= ~option(Parachute_Deployed | Parachute_Expanded | CO2_Asserted | REEF_Asserted);
}

/* Critical conditions */

static inline bool flight_mission_active(void)
{
  return beyond(Armed) || (g_conf & option(Lost_GroundStation)) ||
         fc_recovery_flight_is_admitted();
}

static inline void abortion_due_failures(void)
{
  /* Ordinary flight recovery uses the same lock-safe evaluator handoff.
   * An explicit Stop/reset never resurrects this older autonomous intent. */
  if (!fc_recovery_request_sensor_restart()) return;

  g_conf |= option(Graceful_Reset);
  g_conf &= ~option(Defer_Baro_Fallback);

  smon.gps_delayed = 0;
  smon.gps_malform = 0;

  if (flight_mission_active())
  {
    smon.to_abort = TO_ABORT * 10;

    /* The cooperative control owner will resume after quiescence; no
     * self-blocking send to this owner's potentially full queue. */
  }
  else
  {
    g_conf |= option(In_Aborted_State);
    message(id "aborted; await command", true);
  }
}

static inline void barometer_fallback_vigilant(void)
{
  g_conf &= ~option(GPS_Available);

  baro_conf.osr_p = Baro_OSR_x8;
  baro_conf.iir_coef = Baro_IIR_Coef_15;
  baro_conf.rezero = 0;

  if (g_conf & option(Using_Ascent_KF))
  {
    g_conf |= option(Defer_Baro_Fallback);
    return;
  }

  sensor_init_supervised(Baro_Mask);

  g_conf |= option(Vigilant_Mode);
  g_conf |= option(Measm_Reports);
  message(id "vigilant mode", false);
}


/* Intrusive control */

static inline void manual_deployment(bool apogee, bool force, bool manual)
{
  if (manual ? fc_recovery_inhibits_manual_pyro()
             : fc_recovery_inhibits_pyro() || !fc_watchdog_can_actuate()) {
    if (manual && fc_recovery_reset_any_inflight())
      message(id "Deploy/Expand refused: reset to Startup running", true);
    fc_sd_capture_event(FC_LOG_PYRO_REJECTED, !apogee); return;
  }
  if (apogee)
  {
    /* A refused pulse (reported by release_parachute) keeps the state. */
    if (!release_parachute(force, manual)) return;
    {
      const uint32_t reset_mask = __get_PRIMASK(); __disable_irq();
      fc_recovery_ota_reset_retire_locked();
      sm.flight = Descent;
      __set_PRIMASK(reset_mask);
    }
#ifdef TELEMETRY_ENABLED
    av_bay_underglow_signal(2U);
#else
    blink(Blue, false, 2);
#endif
  }
  else if (expand_parachute(force, manual))
  {
    {
      const uint32_t reset_mask = __get_PRIMASK(); __disable_irq();
      fc_recovery_ota_reset_retire_locked();
      sm.flight = Reefing;
      __set_PRIMASK(reset_mask);
    }
#ifdef TELEMETRY_ENABLED
    av_bay_underglow_signal(4U);
#else
    blink(Blue, false, 4);
#endif
  }
  else return;

  log_metric(id "manual state", sm.flight, true);
  log_flight_state(to_global_state(sm.flight));

  if (g_conf & option(Using_Ascent_KF))
  {
    g_conf &= ~option(Using_Ascent_KF);
    descent_initialize(g_conf);
    /* The estimator may be waiting for an IMU pair the descent filter no
     * longer gets: let it pick up the new mode. */
    (void)tx_event_flags_set(&eval_stage, EVAL_STAGE_WAKE, TX_OR);
  }
}


/* Launch procedures */

static inline void enter_postinit(bool noconfirm)
{
  if (g_conf & option(Postinit_Requested) || beyond(Armed))
  {
    message(id "Postinit blocked: flight/busy", true);
    return;
  }

#ifdef USER_CONFIRMATION

  if (!noconfirm &&
      timer_exchange(PostinitCmd) > CONFIRMATION_TIMEOUT)
  {
    message(id "please confirm postinit");
    return;
  }

#endif

  const uint32_t reset_mask = __get_PRIMASK(); __disable_irq();
  fc_recovery_ota_reset_retire_locked();
  const bool wrong_sequence = ++sm.flight != Postinit;
  if (wrong_sequence) sm.flight = Postinit;
  __set_PRIMASK(reset_mask);
  if (wrong_sequence) message(id "Postinit: wrong sequence", true);

  baro_conf.rezero = 1;
  g_conf |= option(Postinit_Requested);
  message(id "triggered Postinit", true);
}

static inline void enter_launch(bool noconfirm)
{
  if (!beyond(Startup))
  {
    message(id "Launch blocked: no Postinit", true);
    return;
  }

#ifdef USER_CONFIRMATION

  if (!noconfirm &&
      timer_exchange(LaunchCmd) > CONFIRMATION_TIMEOUT)
  {
    message(id "please confirm launch");
    return;
  }

#endif

  if (!fc_recovery_admit_new_flight(noconfirm)) return;

  if (g_conf & option(In_Aborted_State))
  {
    g_conf &= ~option(In_Aborted_State);
    tx_thread_resume(&sensor_thread);
  }
  else if (smon.gps_delayed || smon.gps_malform)
  {
    fi32 total = smon.gps_delayed + smon.gps_malform;
    smon.gps_delayed = 0;
    smon.gps_malform = 0;

    log_metric(id "GPS errors (reset)", total, true);
  }

  smon.failures = 0;
  devctr_if[IMU] = devctr_if[Baro] = 0;
  baro_conf.rezero = 0;
  g_conf &= ~option(Eval_Abort_Flag);
  g_conf &= ~option(Postinit_Requested);

  fc_sd_capture_event(FC_LOG_LAUNCH_ACCEPTED, false);
#if FC_GROUND_TEST
  /* The replay starts only from a validated, pre-Launch entry and a vehicle
   * at rest; the samples here are still the real sensors. */
  {
    float seed[3];
    if (igt_launch(now_ms(), &meas.accl.x, &meas.gyro.x, meas.baro.alt, seed))
      kalman_seed_attitude((const f_xyz *)seed);
  }
#endif
  tx_thread_resume(&estimator_thread);
}


/* Decree execution */

static inline void process_action(fc_msg cmd, bool internal)
{
  switch (cmd) {
    case Postinit_Signal:
      return enter_postinit(internal);

    case Launch_Signal:
      return enter_launch(internal);

#ifdef LUNATIC_STATE

    case Deploy_Parachute:
      return manual_deployment(true, true, !internal);

    case Expand_Parachute:
      return manual_deployment(false, true, !internal);

#else

    /* The operator's Deploy/Expand fires in any flight state so the chutes
     * can be deployed or reefed in an emergency; the ground station gates it
     * by mode and authority. Automatic deployments keep the state checks. */
    case Deploy_Parachute:
      return manual_deployment(true, !internal, !internal);

    case Expand_Parachute:
      return manual_deployment(false, !internal, !internal);

#endif /* LUNATIC_STATE */

    case Reinit_Sensors:
      return sensor_init_supervised(Wild_Mask);

    case Reinit_Barometer:
      return sensor_init_supervised(Baro_Mask);

    case Reinit_IMU:
      return sensor_init_supervised(Gyro_Mask | Accl_Mask);

    case Disable_IMU:
      return sensor_init_supervised(Shut_Gyro | Shut_Accl);

    case Advance_State:
      {
      const uint32_t reset_mask = __get_PRIMASK(); __disable_irq();
      fc_recovery_ota_reset_retire_locked();
      saturating_incr(sm.flight, Recovery);
      __set_PRIMASK(reset_mask);
    }
      break;

    case Rewind_State:
      {
      const uint32_t reset_mask = __get_PRIMASK(); __disable_irq();
      fc_recovery_ota_reset_retire_locked();
      saturating_decr(sm.flight, Startup);
      __set_PRIMASK(reset_mask);
    }
      break;

    case Log_Rate_Limit:
      rates.gnd = LOG_RATE_LIMITED;
      message(id "WARN: reserve heap", true);
      break;

    case Log_Restrict:
      rates.gnd = UINT_FAST32_MAX;
      message(id "WARN: shared stack pool", true);
      break;

    case SD_Retry:
      g_fc_sd_state |= FC_SD_RETRY_PENDING;
#if FC_GROUND_TEST
      igt_sd_retry();
#endif
      break;

#if FC_GROUND_TEST
    case IGT_Enter:
    case IGT_Live_Pyro:
    case IGT_Dry_Run:
    case IGT_Exit:
      if (!internal)
      {
        const fu32 conf = load(&g_conf, Acq);
        const bool pad = current() == Startup &&
            !(conf & option(Postinit_Requested | Launch_Requested | Lost_GroundStation)) &&
            !fc_recovery_flight_is_admitted();
        (void)igt_command((unsigned)(cmd - IGT_Enter), pad,
                          (conf & option(Launch_Requested)) != 0U);
      }
      break;
#endif

    case Log_Terminate:
#ifdef TELEMETRY_ENABLED
      tx_thread_terminate(&telemetry_thread);
      message(id "telemetry stopped", true);
#endif
      break;

    default: break;
  }
}


/* Message processing */

static inline void process_sensor_report(fc_msg code)
{
  bool bad_baro = (code & msmcode(Bad_Altitude)) ||
                  (code & msmcode(Bad_Pressure));

  bool maybe_gps = (g_conf & option(GPS_Available)) &&
                    smon.gps_delayed < GPS_SUS_DELAYS &&
                    smon.gps_malform < GPS_SUS_MALFORM;

  log_metric(id "bad data report", msmcode(code), false);

  smon.failures = devctr_if[IMU] + devctr_if[Baro];

  if (!beyond(Apogee) || (bad_baro && !maybe_gps))
  {
    if (smon.failures >= smon.to_abort)
    {
      abortion_due_failures();
    }
    else if (smon.failures >= smon.to_reinit)
    {
      /* Broad heuristic because Baro takes a while to init.
        */
      sensor_init_supervised(bad_baro ? Wild_Mask
                                      : Gyro_Mask | Accl_Mask);
    }
  }
}

static inline void process_gps_code(fc_msg code)
{
  switch (code)
  {
  case GPS_Delayed:
    ++smon.gps_delayed;

    if (g_conf & option(Launch_Requested))
    {
      log_metric(id "#delayed GPS", smon.gps_delayed, false);
    }
    break;

  case GPS_Malformed:
    ++smon.gps_malform;

    if (g_conf & option(Launch_Requested))
    {
      log_metric(id "#malform GPS", smon.gps_malform, false);
    }
    break;

  default: return;
  }

  if (smon.gps_delayed >= GPS_MAX_DELAYS ||
      smon.gps_malform >= GPS_MAX_MALFORM)
  {
    barometer_fallback_vigilant();
  }
}

/* Retired decrees: EvaluationRelax/Focus switched the evaluator's
 * preemption, which the estimator guard replaced; the operator's Evaluation
 * Abort is now part of the reset to Startup (fc_recovery_control_finalize_
 * startup). An old station's copy is refused (and acknowledged, so it is not
 * retransmitted). */
static inline bool retired_decree(fc_msg cmd)
{
  return cmd == Retired_Evaluation_Relax || cmd == Retired_Evaluation_Focus ||
         cmd == Retired_Evaluation_Abort;
}

bool fc_recovery_command_admissible(uint32_t command, bool internal)
{
  const fc_msg cmd = (fc_msg)command;
  if (retired_decree(cmd)) return false;
  /* The station can no longer set, revoke or tune any runtime option or
   * failure threshold: those commands are retired and the FC keeps its
   * compiled defaults (USER_OPTIONS, TO_ABORT, TO_REINIT). */
  if (!internal && (cmd & Runtime_Configuration)) return false;
  /* The operator's emergency Deploy/Expand passes every sequence hold. */
  if (!internal && (cmd == Deploy_Parachute || cmd == Expand_Parachute))
    return !fc_recovery_inhibits_manual_pyro();
  if (fc_recovery_holds_sequence()) {
    /* Only the operator's deploy commands may pass a rejected watchdog resume. */
    return !internal && (cmd == Deploy_Parachute || cmd == Expand_Parachute) &&
           !fc_recovery_inhibits_manual_pyro();
  }
  if (!fc_recovery_requires_new_flight()) return true;
  switch (cmd) {
    case Launch_Signal: return !internal;
    case Deploy_Parachute: case Expand_Parachute:
    case Advance_State: case Rewind_State: return false;
    default: return true;
  }
}

static inline void decode_flight_message(fc_msg msg)
{
  bool internal = msg == fc_mask(msg);
  if (!fc_recovery_command_admissible((uint32_t)fc_unmask(msg), internal)) return;

  if (internal)
  {
    msg = fc_unmask(msg);

    if (msg & Sensor_Measm_Code)
    {
      return process_sensor_report(msg);
    }
    else if (msg & GPS_Data_Code)
    {
      return process_gps_code(msg);
    }
  }

  if (msg & Actionable_Decrees)
  {
    return process_action(msg, internal);
  }

  log_metric(id "unknown option", (fi32) fc_unmask(msg), true);
}


/* Scheduler-managed routines */

static inline conditional void autostart_on_timeout(void)
{
#ifndef FAKESTATION
  if (fc_recovery_inhibits_pyro()) return;
  static fu8 test_stage = 0;

  if (test_stage < 2)
  {
    ++test_stage;
  } 
  else if (test_stage == 2)
  {
    ++test_stage;
    fc_msg cmd = fc_mask(Postinit_Signal);
    thread_comm_send(cmd, TX_NO_WAIT);
  }
  else if (test_stage < 4)
  {
    ++test_stage;
  }
  else if (test_stage == 4)
  {
    ++test_stage;
    fc_msg cmd = fc_mask(Launch_Signal);
    thread_comm_send(cmd, TX_NO_WAIT);
  }
#endif
}

/* Ground station heartbeat loss and GPS gap. Runs in the control thread
 * every HEALTH_CHECK_TICKS (health_service below); it used to run as a
 * ThreadX timer inside the SysTick interrupt. */
static void health_check(void)
{
  sweetbench_catch(6);

  if (timer_fetch(HeartbeatGND) > GND_TIMEOUT)
  {
#ifdef TELEMETRY_ENABLED
    g_conf |= option(Lost_GroundStation);
    g_conf |= option(Reset_Failures);
    /* No self-Launch while aborted: it resumed evaluation from an unadmitted
     * internal launch and requested an ignition no board acts on. */
#else
    autostart_on_timeout();
#endif
  }
#ifdef TELEMETRY_ENABLED
  /* On the pad the loss clears once the heartbeat returns; in flight it holds. */
  else if ((g_conf & option(Lost_GroundStation)) && !beyond(Armed) &&
           !fc_recovery_flight_is_admitted())
  {
    g_conf &= ~option(Lost_GroundStation);
  }
#endif

  fu32 gps_interval = timer_fetch(HeartbeatRF);

  if (g_conf & option(GPS_Available))
  {
    if (gps_interval > GPS_DELAY_MS)
    {
      fc_msg cmd = fc_mask(GPS_Delayed);
      thread_comm_send(cmd, TX_NO_WAIT);
    }
  }
  else if (gps_interval < HEALTH_CHECK_TICKS)
  {
    g_conf |= option(GPS_Available);
  }

  sweetbench_start(6, 10);
}


/* The same schedule as the old timer: first HEALTH_CHECK_FIRST_TICKS after
 * the command loop starts, then every HEALTH_CHECK_TICKS on a fixed grid. A
 * check delayed by a command still runs once per missed slot. */
static bool health_active;
static ULONG health_due;

static void health_start(void)
{
  health_due = tx_time_get() + HEALTH_CHECK_FIRST_TICKS;
  health_active = true;
}

static void health_service(void)
{
  while (health_active && (LONG)(tx_time_get() - health_due) >= 0)
  {
    health_check();
    health_due += HEALTH_CHECK_TICKS;
  }
}

/* The longest the command wait may block before the next check is due. */
static ULONG health_wait(ULONG limit)
{
  const ULONG left = health_due - tx_time_get();
  return (LONG)left <= 0 ? TX_NO_WAIT : (left < limit ? left : limit);
}


/* Task */

void control_thread_entry(ULONG st)
{
  /* The recovery task has the highest priority. Give the telemetry worker one
   * scheduler tick to initialize SEDSNet before recovery emits its boot-state
   * network log; otherwise recovery can initialize the router recursively and
   * prevent every lower-priority task from starting. */
  tx_thread_sleep(1U);
  try_allocate_reserve_pool();

  if (!g_fc_watchdog_resumed)
  for (timer k = 0; k < Time_Users; ++k)
  {
    timer_update(k);
  }

  local_time[PostinitCmd] = UINT_FAST32_MAX;
  local_time[LaunchCmd] = UINT_FAST32_MAX;

  if (g_fc_watchdog_resumed)
    baro_conf.rezero = 0; /* retain the launch pressure, even above ground */
  /* Sensor telemetry remains available when recovery rejects actuation. */
  sensor_init_supervised(Wild_Mask);
  if (g_fc_watchdog_resumed && !fc_recovery_inhibits_pyro()) {
    if (current() >= Descent && (g_conf & option(Using_Ascent_KF)))
      descent_initialize(g_conf);
    timer_update(AscentKF);
    timer_update(DescentKF);
    /* The estimator runs both filters: resume it in either phase. */
    tx_thread_resume(&estimator_thread);
  }
  health_start();

  /* Flight state is restored before ThreadX starts and then converges through
   * its managed network variable. Publishing it here lets this highest-priority
   * task preempt telemetry while telemetry is inside discovery, then enter
   * the same router from a second thread. Under a busy CAN topology that can
   * deadlock both tasks immediately after a reboot. Leave all router work to
   * the telemetry task; later real state transitions still publish normally. */
  g_recovery_receive_loop_ready = 1U;

  for (;;)
  {
    thread_comm_msg_t msg;
    health_service();
    fc_monitor_bench_poll();
    /* Every command and control step runs under the estimator guard, so it
     * never lands in the middle of an evaluation pass (thread_comm.h). The
     * guard is never held across the waits below. */
    thread_comm_estimator_enter();
    fc_recovery_control_service();
    thread_comm_estimator_leave();
    if (fc_recovery_take_sensor_restart()) {
      thread_comm_estimator_enter();
      if (flight_mission_active())
        decode_flight_message(fc_mask(Launch_Signal));
      thread_comm_estimator_leave();
      distribution_wake();
      telemetry_wake();
      continue;
    }
    if (fc_recovery_control_pending()) {
      board_watchdog_progress(BOARD_WATCHDOG_CONTROL);
      tx_thread_sleep(1U); /* Let evaluator/telemetry acknowledge; never spin. */
      continue;
    }
    const uint32_t command_epoch = fc_recovery_epoch();

    recovery_update_stack_profile();
    /* Thread suspension */
    board_watchdog_progress(BOARD_WATCHDOG_CONTROL);
    st = thread_comm_receive(&msg, health_wait(TX_TIMER_TICKS_PER_SECOND / 4U));

    if (st != TX_SUCCESS)
    {
      continue;
    }

    thread_comm_estimator_enter();
    fc_recovery_control_service();
    thread_comm_estimator_leave();
    if (command_epoch != fc_recovery_epoch()) continue;
    g_recovery_commands_dequeued++;
    const bool network_command = msg.origin == THREAD_COMM_NETWORK;
    thread_comm_estimator_enter();
    decode_flight_message((fc_msg)msg.cmd);
    thread_comm_estimator_leave();
    /* Commands change the configuration distribution's loops poll and the
     * latches (Abort status) the telemetry loop publishes. */
    distribution_wake();
    telemetry_wake();
    recovery_update_stack_profile();
    if (network_command)
    {
      g_network_flight_commands_processed++;
    }
  }
}

UINT create_control_thread(TX_BYTE_POOL *byte_pool)
{
  UINT st;
  CHAR *pointer;

  tx_app_shared = byte_pool;
  const char *critical = "creation failure:";

  st = tx_byte_allocate(byte_pool, (VOID **)&pointer,
                        CONTROL_STACK_BYTES, TX_NO_WAIT);

  if (st != TX_SUCCESS)
  {
    log_err(id "stack %s %u", critical, st);
    return FC_INIT_FAILED;
  }
  
  /* The command queue (thread_comm_init) exists before any thread starts:
   * with TX_AUTO_START this highest-priority thread can preempt its creator
   * immediately, and an uncreated queue would make its receive fail in a
   * tight loop, starving telemetry and every other task. */
  st = tx_thread_create(&control_thread,
                        "Control",
                        control_thread_entry,
                        CONTROL_INPUT,
                        pointer,
                        CONTROL_STACK_BYTES,
                        CONTROL_PRIORITY,
                        /* No preemption threshold */
                        CONTROL_PRIORITY,
                        TX_NO_TIME_SLICE,
                        TX_AUTO_START);

  if (st != TX_SUCCESS)
  {
    log_err(id "task %s %u", critical, st);
    return FC_INIT_FAILED;
  }

  return TX_SUCCESS;
}
