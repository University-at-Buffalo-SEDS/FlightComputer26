/*
 * Recovery Task
 */

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
    .iir_coef = Baro_IIR_Coef_0,
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
  { Eval_Abort_Flag,     "eval-abort" },
  { Reset_Failures,      "reset-failures" },
  { Validate_Measms,     "validate-measms" },
};


/*
 * Tries to (re)initialize or shut given sensors.
 * Should never be preemtped by other task.
 */
static inline void initialize_sensors(sens_init sn)
{
  sens_init fails = 0;

  sweetbench_start(5, 1);

  clear_spi1_irq();

  if (sn & Init_Baro)
  {
    try_init_sensor(baro_init(&hspi1, &baro_conf),
                                fails, Init_Baro);
  }

  if (sn & Init_Gyro)
  {
    try_init_sensor(gyro_init(&hspi1, &gyro_conf),
                                fails, Init_Gyro);
  }

  if (sn & Init_Accl)
  {
    try_init_sensor(accl_init(&hspi1, &accl_conf),
                                fails, Init_Accl);
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

/*
 * Resets Distribution and Evaluation tasks.
 * If GroundStation is lost, restarts them.
 */
static inline void auto_abort(void)
{
  tx_thread_reset(&evaluation_task);
  tx_thread_reset(&distribution_task);

  smon.gps_delay = 0;
  smon.gps_malform = 0;
  g_conf &= ~option(Defer_Baro_Fallback);

  if (g_conf & option(Lost_GroundStation))
  {
    smon.to_abort = TO_ABORT * 10;

    fc_msg cmd = fc_mask(Launch_Signal);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }
  else
  {
    log_msg(id "Aborted! Expecting commands.");

    g_conf |= option(In_Aborted_State);
  }
}

/*
 * Signals other tasks to not fetch or use GPS.
 * Reinitializes barometer with larger OSR.
 */
static inline void barometer_fallback(void)
{
  g_conf &= ~option(GPS_Available);

  baro_conf.osr_p = Baro_OSR_x8;
  baro_conf.iir_coef = Baro_IIR_Coef_15;

  if (g_conf & option(Using_Ascent_KF))
  {
    g_conf |= option(Defer_Baro_Fallback);
    return;
  }

  initialize_sensors(Init_Baro);

  g_conf |= option(Monitor_Altitude);
  g_conf |= option(Validate_Measms);
  log_msg(id "Entered vigilant mode");
}

/*
 * Process general command from either endpoint.
 */
static inline void process_action(fc_msg cmd)
{
  UINT eval_old_pt;

  switch (cmd) {
    case Deploy_Parachute:
      flight = Descent;

      release_parachute();
      log_transition(id, sv[sm.idx].alt);

      if (g_conf & option(Using_Ascent_KF))
      {
        g_conf &= ~option(Using_Ascent_KF);
        descent_initialize();
      }
      return;

  case Expand_Parachute:
    if (expand_parachute())
    {
      flight = Reefing;
      log_transition(id, sv[sm.idx].alt);
    }
    return;

  case Reinit_Sensors:
    initialize_sensors(Init_All);
    return;

  case Launch_Signal:
    if (g_conf & option(In_Aborted_State))
    {
      g_conf &= ~option(In_Aborted_State);
      tx_thread_resume(&distribution_task);
    }
    else if (smon.gps_delay || smon.gps_malform)
    {
      log_err(id "GPS: %u delayed and %u malformed "
                 "during pre-launch. Counters are reset.",
              smon.gps_delay, smon.gps_malform);

      smon.gps_delay = 0;
      smon.gps_malform = 0;
    }

    g_conf &= ~option(Eval_Abort_Flag);
    tx_thread_resume(&evaluation_task);
    return;

  case Evaluation_Relax:
    g_conf &= ~option(Eval_Focus_Flag);
    tx_thread_preemption_change(&evaluation_task,
                                EVAL_PRIORITY,
                                &eval_old_pt);
    return;

  case Evaluation_Focus:
    g_conf |= option(Eval_Focus_Flag);
    tx_thread_preemption_change(&evaluation_task,
                                EVAL_PREEMPT_THRESHOLD,
                                &eval_old_pt);
    return;

  case Evaluation_Abort:
    g_conf |= option(Eval_Abort_Flag);
    tx_thread_reset(&evaluation_task);
    return;

  case Reinit_Barometer:
    initialize_sensors(Init_Baro);
    return;

  case Reinit_IMU:
    initialize_sensors(Init_Gyro | Init_Accl);
    return;

  case Disable_IMU:
    initialize_sensors(Shut_Gyro | Shut_Accl);
    return;

  case Advance_State:
    if (flight + 1 < Flight_States)
    {
      ++flight;
    }
    break;

  case Rewind_State:
    if (flight - 1 >= Suspended)
    {
      --flight;
    }
    break;

  default:
    break;
  }
}

/*
 * Constant-folded mask of all possible user options.
 * Allows to extend enum message without modifying code.
 */
static inline constexpr fc_msg user_options()
{
  fc_msg options = 0;

  for (fu32 k = 1; k < option(User_Option_Bound); k *= 2)
  {
    options |= k;
  }

  return options;
}

/*
 * Updates global configuration and lists all currently
 * set options.
 */
static inline void update_config(fc_msg incoming)
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
  else
  {
    g_conf |= raw;
  }

  int cursor = sizeof(id) + 9 - 1;
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

/*
 * Applies or revokes passed runtime configuration option.
 */
static inline void process_config(fc_msg code)
{
  if (code & Abortion_Thresholds)
  {
    smon.to_abort = threshold(code & ~Abortion_Thresholds);
  }
  else if (code & Reinit_Thresholds)
  {
    smon.to_reinit = threshold(code & ~Reinit_Thresholds);
  }
  else
  {
    update_config(option(code));
  }
}

/*
 * Checks whether raw data report is OK.
 */
static inline void process_report(fc_msg code)
{
  if (code != Sensor_Measm_Code)
  {
    bool bad_baro = (code & Bad_Altitude) || (code & Bad_Pressure);

    bool maybe_gps = (g_conf & option(GPS_Available)) &&
                      smon.gps_delay < GPS_SUS_DELAYS &&
                      smon.gps_malform < GPS_SUS_MALFORM;

    log_err(id "dirty data report: %u", (unsigned)code);

    if (flight <= Apogee || (bad_baro && !maybe_gps))
    {
      ++smon.failures;

      if (smon.failures >= smon.to_abort)
      {
        auto_abort();
      }
      else if (smon.failures >= smon.to_reinit)
      {
        /* Broad heuristic because Baro takes a while to init.
         */
        bad_baro ? initialize_sensors(Init_All)
                 : initialize_sensors(Init_Gyro | Init_Accl);
      }
    }
  }
  else if (g_conf & option(Reset_Failures))
  {
    smon.failures = 0;
  }
}

/*
 * Handles delayed or malformed GPS data report.
 */
static inline void process_gps_code(fc_msg code)
{
  switch (code)
  {
  case GPS_Delayed:
    ++smon.gps_delay;

    if (g_conf & option(Launch_Triggered))
    {
      log_err(id "delayed GPS packets: %u", smon.gps_delay);
    }
    break;

  case GPS_Malformed:
    ++smon.gps_malform;

    if (g_conf & option(Launch_Triggered))
    {
      log_err(id "malformed GPS packets: %u", smon.gps_malform);
    }
    break;

  default: return;
  }

  if (smon.gps_delay >= GPS_MAX_DELAYS ||
      smon.gps_malform >= GPS_MAX_MALFORM)
  {
    barometer_fallback();
  }
}

/*
 * Decodes universal FC message from an endpoint {FC, RF, GND}.
 */
static inline void decode_message(fc_msg msg)
{
  if (msg & FlightComputer_Mask)
  {
    msg = fc_unmask(msg);

    if (msg < Spurious_Confirmations)
    {
      return process_report(msg);
    }
    else if (msg & GPS_Data_Code)
    {
      return process_gps_code(msg);
    }
    else if (msg & Spurious_Confirmations)
    {
      msg &= ~Spurious_Confirmations;
      log_err(id "unconfirmed transition: %u", (unsigned)msg);
      return;
    }
  }

  if (msg & GroundStation_Heartbeat)
  {
    sweetbench_catch(11);
    timer_update(HeartbeatGND);
    sweetbench_start(11, 100);
  }
  else if (msg & Actionable_Decrees)
  {
    process_action(msg);
  }
  else if (msg & Runtime_Configuration)
  {
    process_config(msg);
  }
  else
  {
    log_err(id "slap yourself %u times", (unsigned)msg);
  }
}


/*
 * Monotonic routine. Monitors for endpoint timeouts
 * and deactivates open deployment channels.
 */
static void fc_timer_routine(ULONG timer_id)
{
  (void)timer_id;

  sweetbench_catch(6);

  if (g_conf & option(CO2_Asserted) &&
      timer_fetch(AssertCO2) >= CO2_ASSERT_INTERVAL)
  {
    co2_low();
    g_conf &= ~option(CO2_Asserted);
  }

  if (g_conf & option(REEF_Asserted) &&
      timer_fetch(AssertREEF) >= REEF_ASSERT_INTERVAL)
  {
    reef_low();
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
      tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
    }

#else
    static fu8 test_launched = 0;

    if (!test_launched)
    {
      test_launched = 1;
      enum message cmd = fc_mask(Launch_Signal);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }

#endif /* TELEMETRY_ENABLED */
  }

  if (g_conf & option(GPS_Available))
  {
    if (timer_fetch(HeartbeatRF) > GPS_DELAY_MS)
    {
      fc_msg cmd = fc_mask(GPS_Delayed);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }
  }
  else
  {
    if (timer_fetch(HeartbeatRF) < TX_TIMER_TICKS)
    {
      g_conf |= option(GPS_Available);
    }
  }

  sweetbench_start(6, 10);
}


/*
 * This task is responsible for global configuration
 * and starting other dependent tasks.
 */
void recovery_entry(ULONG input)
{
  (void)input;

  log_msg(id "started");

  UINT st;

  initialize_sensors(Init_All);

  for (timer k = 0; k < Time_Users; ++k)
  {
    timer_update(k);
  }

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

    decode_message(msg);
  }
}

/*
 * Creates a non-preemptive Recovery Task with defined parameters.
 */
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

  st = tx_queue_create(&shared, "RECVQ", 1, &recvq,
                       sizeof recvq);

  if (st != TX_SUCCESS)
  {
    log_die(id "queue %s %u", critical, st);
  }

  st = tx_timer_create(&monotonic_checks, "RECVT",
                       fc_timer_routine, 0, TX_TIMER_INITIAL,
                       TX_TIMER_TICKS, TX_NO_ACTIVATE);

  if (st != TX_SUCCESS)
  {
    log_die(id "timer %s %u", critical, st);
  }

  return TX_SUCCESS;
}