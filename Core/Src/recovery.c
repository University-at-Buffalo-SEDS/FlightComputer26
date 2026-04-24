/* Core/Src/recovery.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"

#define id "RE "


TX_THREAD recovery_task;
TX_QUEUE shared;
TX_TIMER monotonic_checks;

volatile fu32 local_time[Time_Users] = {0};

static tx_align fc_msg recvq[FC_MSG_Q_SIZE] = {0};

atomic_uint_fast32_t g_conf = FC_DEFAULTS | USER_OPTIONS;

static sysmon smon = {TO_ABORT, TO_REINIT, 0, 0, 0};

static struct baro_config baro_conf = {
    .osr_t = Baro_OSR_x1,
    .osr_p = Baro_OSR_x2,
    .odr = BARO_DEFAULT_ODR_SEL,
    .iir_coef = Baro_IIR_Coef_3,
};

static struct gyro_config gyro_conf = {
    .rng = Gyro_Range_2000Dps,
    .bw = Gyro_32Hz_ODR_100Hz,
};

static struct accl_config accl_conf = {
    .mode = Normal_100Hz,
    .rng = Accl_Range_24g,
};

static const conf_dict confmap[] = {
  { Monitor_Altitude,    "vigilant-mode" },
  { Consecutive_Samples, "consectuive-samples" },
  { Eval_Focus_Flag,     "eval-focus" },
  { Reset_Failures,      "reset-failures" },
  { Measm_Reports,   "report-bad-measms" },
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
    log_err(id "observed init failures: %u", fails);
  }

  sweetbench_catch(5);
}


/* Critical conditions */

static inline void abortion_due_failures(void)
{
  tx_thread_terminate(&evaluation_task);
  tx_thread_reset(&evaluation_task);

  g_conf |= option(Graceful_Reset);
  g_conf &= ~option(Defer_Baro_Fallback);

  smon.gps_delay = 0;
  smon.gps_malform = 0;

  if (beyond(Postinit) ||
      g_conf & option(Lost_GroundStation))
  {
    smon.to_abort = TO_ABORT * 10;

    fc_msg cmd = fc_mask(Launch_Signal);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }
  else
  {
    g_conf |= option(In_Aborted_State);
    log_critical(id "aborted, expecting commands");
  }
}

static inline void barometer_fallback_vigilant(void)
{
  g_conf &= ~option(GPS_Available);

  baro_conf.osr_p = Baro_OSR_x8;
  baro_conf.iir_coef = Baro_IIR_Coef_15;

  if (g_conf & option(Using_Ascent_KF))
  {
    g_conf |= option(Defer_Baro_Fallback);
    return;
  }

  sensor_init_supervised(Baro_Mask);

  g_conf |= option(Monitor_Altitude);
  g_conf |= option(Measm_Reports);
  log_msg(id "entered vigilant mode");
}


/* Intrusive control */

static inline void evaluation_configure(bool focus)
{
  UINT eval_old_pt, eval_new_pt;

  if (g_conf & option(Eval_Abort_Flag))
  {
    g_conf &= ~option(Eval_Abort_Flag);
    tx_thread_resume(&evaluation_task);
    log_msg(id "restored evaluation");
  }

  if (focus)
  {
    eval_new_pt = EVAL_PREEMPT_THRESHOLD;
    g_conf |= option(Eval_Focus_Flag);
  }
  else
  {
    eval_new_pt = EVAL_PRIORITY;
    g_conf &= ~option(Eval_Focus_Flag);
  }

  tx_thread_preemption_change(&evaluation_task,
                              eval_new_pt,
                              &eval_old_pt);
}

static inline void manual_deployment(bool apogee)
{
  if (apogee)
  {
    sm.flight = Descent;
    release_parachute();
    blink(LED_BLINKS_ON_CO2);
  }
  else if (expand_parachute())
  {
    sm.flight = Reefing;
    blink(LED_BLINKS_ON_REEF);
  }
  else return;

  log_flight_state(sm.flight);

  if (g_conf & option(Using_Ascent_KF))
  {
    g_conf &= ~option(Using_Ascent_KF);
    descent_initialize();
  }
}


/* Launch procedures */

static inline void enter_postinit(bool noconfirm)
{
  if (beyond(Launch))
  {
    log_err(id "rejected postinit mid-flight");
    return;
  }

  if (!noconfirm &&
      timer_exchange(PostinitCmd) > CONFIRMATION_TIMEOUT)
  {
    log_critical(id "please confirm postinit");
    return;
  }

  if (satur_incr(sm.flight, Landed) != Postinit)
  {
    sm.flight = Postinit;
    log_err(id "unusual sequence at postinit");
  }

  g_conf |= option(Postinit_Requested);
}

static inline void enter_launch(bool noconfirm)
{
  if (!noconfirm &&
      timer_exchange(LaunchCmd) > CONFIRMATION_TIMEOUT)
  {
    log_critical(id "please confirm launch");
    return;
  }

  if (g_conf & option(In_Aborted_State))
  {
    g_conf &= ~option(In_Aborted_State);
    tx_thread_resume(&distribution_task);
  }
  else if (smon.gps_delay || smon.gps_malform)
  {
    log_err(id "GPS: %u delayed and %u malformed "
               "during pre-launch, now reset.",
            smon.gps_delay, smon.gps_malform);

    smon.gps_delay = 0;
    smon.gps_malform = 0;
  }

  smon.failures = 0;
  g_conf &= ~option(Eval_Abort_Flag);

  tx_thread_resume(&evaluation_task);
}

static inline void rollback_to_idle(void)
{
  if (timer_exchange(RollbackCmd) > CONFIRMATION_TIMEOUT)
  {
    if (g_conf & option(Launch_Requested))
    {
      log_err(id "looks like we're flying. ARE YOU SURE?");
    }

    log_critical(id "please confirm rollback");
    return;
  }

  g_conf |= option(Rollback_Requested);
  g_conf &= ~option(Postinit_Requested);
  g_conf &= ~option(Launch_Requested);
  g_conf &= ~option(Measm_Reports);

  sm.flight = Suspended;

  log_msg(id "rolled back to pre-init");
}


/* Decree execution */

static inline void process_action(fc_msg cmd, bool internal)
{
  switch (cmd) {
    case Postinit_Signal:
      return enter_postinit(internal);

    case Launch_Signal:
      return enter_launch(internal);

    case Rollback_Signal:
      return rollback_to_idle();

    case Deploy_Parachute:
      return manual_deployment(true);

    case Expand_Parachute:
      return manual_deployment(false);

    case Reinit_Sensors:
      return sensor_init_supervised(Wild_Mask);

    case Reinit_Barometer:
      return sensor_init_supervised(Baro_Mask);

    case Reinit_IMU:
      return sensor_init_supervised(Gyro_Mask | Accl_Mask);

    case Disable_IMU:
      return sensor_init_supervised(Shut_Gyro | Shut_Accl);

    case Evaluation_Relax:  
      return evaluation_configure(false);

    case Evaluation_Focus:
      return evaluation_configure(true);

    case Evaluation_Abort:
      g_conf |= option(Eval_Abort_Flag);
      tx_thread_terminate(&evaluation_task);
      tx_thread_reset(&evaluation_task);
      return;

    case Advance_State:
      satur_incr(sm.flight, Landed);
      break;

    case Rewind_State:
      satur_decr(sm.flight, Suspended);
      break;

    default: break;
  }
}


/* Global config */

static inline constexpr fc_msg user_options()
{
  fc_msg options = 0;
  fu32 k = 1;

  for (; k < option(User_Option_Bound); k *= 2)
  {
    options |= k;
  }

  return options;
}

static inline void update_global_config(fc_msg incoming)
{
  const fc_msg valid = user_options();
  fu32 raw = incoming & ~Revoke_Option;

  if ((raw & valid) == 0 || (raw & ~valid) != 0 ||
      (raw & (raw - 1)) != 0)
  {
    log_err(id "option ill-formed: %u", (unsigned)incoming);
    return;
  }

  if (incoming & Revoke_Option)
  {
    g_conf &= ~raw;
  }
  else g_conf |= raw;

  int cursor = sizeof(id) + 8;
  char buf[MAX_CONFIG_REPORT_SIZE] = id "options: ";

  for (fu16 k = 0; k < namecount(confmap); ++k)
  {
    if ((g_conf & confmap[k].val) == option(confmap[k].val))
    {
      cursor += snprintf(buf + cursor,
                         sizeof confmap[k].name,
                         "%s ",
                         confmap[k].name);
    }
  }

  log_msg(buf);
}


/* Message processing */

static inline void process_config_update(fc_msg code)
{
  if (code & Abortion_Thresholds)
  {
    smon.to_abort = threshold(code & ~Abortion_Thresholds);
  }
  else if (code & Reinit_Thresholds)
  {
    smon.to_reinit = threshold(code & ~Reinit_Thresholds);
  }
  else update_global_config(option(code));
}

static inline void process_sensor_report(fc_msg code)
{
  if (code != Sensor_Measm_Code)
  {
    bool bad_baro = (code & msmcode(Bad_Altitude)) ||
                    (code & msmcode(Bad_Pressure));

    bool maybe_gps = (g_conf & option(GPS_Available)) &&
                      smon.gps_delay < GPS_SUS_DELAYS &&
                      smon.gps_malform < GPS_SUS_MALFORM;

    log_err(id "bad data report: %u", msmcode(code));

    if (!beyond(Apogee) || (bad_baro && !maybe_gps))
    {
      if (++smon.failures >= smon.to_abort)
      {
        abortion_due_failures();
      }
      else if (smon.failures >= smon.to_reinit)
      {
        /* Broad heuristic because Baro takes a while to init.
         */
        bad_baro ? sensor_init_supervised(Wild_Mask)
                 : sensor_init_supervised(Gyro_Mask | Accl_Mask);
      }
    }
  }
  else smon.failures = 0;
}

static inline void process_gps_code(fc_msg code)
{
  switch (code)
  {
  case GPS_Delayed:
    ++smon.gps_delay;

    if (g_conf & option(Launch_Requested))
    {
      log_err(id "delayed GPS packets: %u", smon.gps_delay);
    }
    break;

  case GPS_Malformed:
    ++smon.gps_malform;

    if (g_conf & option(Launch_Requested))
    {
      log_err(id "malformed GPS packets: %u", smon.gps_malform);
    }
    break;

  default: return;
  }

  if (smon.gps_delay >= GPS_MAX_DELAYS ||
      smon.gps_malform >= GPS_MAX_MALFORM)
  {
    barometer_fallback_vigilant();
  }
}

static inline void decode_flight_message(fc_msg msg)
{
  bool internal = msg == fc_mask(msg);

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
  else if (msg & Runtime_Configuration)
  {
    return process_config_update(msg);
  }

  log_err(id "unrecognized option: %u", (fu32) msg);
}


/* Scheduler-managed routines */

static void fc_timer_routine(ULONG timer_id)
{
  (void)timer_id;

  sweetbench_catch(6);

  if (g_conf & option(CO2_Asserted) &&
      timer_fetch(AssertCO2) >= CO2_ASSERT_INTERVAL)
  {
    co2_low();
    sweetbench_catch(4);
    g_conf &= ~option(CO2_Asserted);
  }

  if (g_conf & option(REEF_Asserted) &&
      timer_fetch(AssertREEF) >= REEF_ASSERT_INTERVAL)
  {
    reef_low();
    sweetbench_catch(4);
    g_conf &= ~option(REEF_Asserted);
  }

  if (timer_fetch(HeartbeatGND) > GND_TIMEOUT)
  {
#ifdef TELEMETRY_ENABLED
    g_conf |= option(Lost_GroundStation);
    g_conf |= option(Monitor_Altitude);
    g_conf |= option(Reset_Failures);
    smon.failures = 0;

    if (g_conf & option(In_Aborted_State))
    {
      smon.to_abort = smon.to_abort * 10;

      fc_msg cmd = fc_mask(Launch_Signal);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }

#else
    static fu8 test_stage = 0;

    if (test_stage < 2)
    {
      ++test_stage;
    } 
    else if (test_stage == 2)
    {
      ++test_stage;
      fc_msg cmd = fc_mask(Postinit_Signal);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }
    else if (test_stage < 4)
    {
      ++test_stage;
    }
    else if (test_stage == 4)
    {
      ++test_stage;
      fc_msg cmd = fc_mask(Launch_Signal);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }

#endif /* TELEMETRY_ENABLED */
  }

  fu32 gps_interval = timer_fetch(HeartbeatRF);

  if (g_conf & option(GPS_Available) &&
      gps_interval > GPS_DELAY_MS)
  {
    fc_msg cmd = fc_mask(GPS_Delayed);
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
  else if (gps_interval < TX_TIMER_TICKS)
  {
    g_conf |= option(GPS_Available);
  }

  sweetbench_start(6, 10);
}

static void grace_reset_distribution(TX_THREAD *ptr, UINT cond)
{
  if (ptr != &distribution_task ||
      (g_conf & option(Graceful_Reset)) == 0)
  {
     return;
  }

  if (cond == TX_THREAD_EXIT)
  {
    tx_thread_reset(&distribution_task);

    if (g_conf & option(Rollback_Requested))
    {
      g_conf &= ~option(Rollback_Requested);
      sensor_init_supervised(Wild_Mask);
      timer_update(FillSequence);
      tx_thread_resume(&distribution_task);
    }
  }
  else log_msg(id "(re)started distribution");
}


/* Task */

void recovery_entry(ULONG input)
{
  (void)input;

  UINT st;
  
  st = tx_thread_entry_exit_notify(&distribution_task,
                                   grace_reset_distribution);

  if (st != TX_SUCCESS)
  {
    log_die(id "get off me: %u", st);
  }

  sensor_init_supervised(Wild_Mask);

  for (timer k = 0; k < Time_Users; ++k)
  {
    timer_update(k);
  }

  local_time[PostinitCmd] = UINT_FAST32_MAX;
  local_time[LaunchCmd] = UINT_FAST32_MAX;
  local_time[RollbackCmd] = UINT_FAST32_MAX;

  tx_timer_activate(&monotonic_checks);

  task_loop(DO_NOT_EXIT)
  {
    fc_msg msg;

    /* Thread suspension */
    st = tx_queue_receive(&shared, &msg, TX_WAIT_FOREVER);

    if (st != TX_SUCCESS)
    {
      continue;
    }

    decode_flight_message(msg);
  }
}

UINT create_recovery_task(TX_BYTE_POOL *byte_pool)
{
  UINT st;
  CHAR *pointer;

  /* Allocate the stack for test  */
  if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                       RECV_STACK_BYTES, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  
  st = tx_thread_create(&recovery_task,
                        "Recovery Task",
                        recovery_entry,
                        RECV_INPUT,
                        pointer,
                        RECV_STACK_BYTES,
                        RECV_PRIORITY,
                        /* No preemption threshold */
                        RECV_PRIORITY,
                        TX_NO_TIME_SLICE,
                        TX_AUTO_START);

  const char *critical = "creation failure:";

  if (st != TX_SUCCESS)
  {
    log_die(id "task %s %u", critical, st);
  }

  st = tx_queue_create(&shared, id "Q", 1, &recvq,
                       sizeof recvq);

  if (st != TX_SUCCESS)
  {
    log_die(id "queue %s %u", critical, st);
  }

  st = tx_timer_create(&monotonic_checks, id "T",
                       fc_timer_routine, 0, TX_TIMER_INITIAL,
                       TX_TIMER_TICKS, TX_NO_ACTIVATE);

  if (st != TX_SUCCESS)
  {
    log_die(id "timer %s %u", critical, st);
  }

  return TX_SUCCESS;
}