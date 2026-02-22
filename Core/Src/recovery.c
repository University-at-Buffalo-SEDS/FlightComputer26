/*
 * Recovery Task
 *
 * This task suspends on a blocking message queue
 * and when resumed, drains the message queue. The
 * messages are then interpreted depending on the
 * MSB designating the endpoint - FC or GND. When
 * a message appears in the queue, this thread is 
 * _immediately_ woken by ThreadX and processing begins.
 *
 * This task provides for various recovery and abortion
 * scenarios, and also logs all important actions. This
 * task as a sole command and decision center of the
 * flight computer, and therefore cannot be preempted
 * while in execution. As such, utlizing the fact that
 * the target MCU is single core and thus has only one
 * data cache unit, this task can perform non-atomic
 * reads and writes across its allotted RAM region. 
 *
 * This task also includes a timer callback that
 * checks for timeout of FC, RF and GND. If timeout
 * occurs (i.e., this task did not receive a signal)
 * for 4 seconds, a handler is invoked that changes
 * global runtime configuration, which other tasks
 * read and locally update on every iteration. The
 * ThreadX timer is set to expire every second.
 *
 * To change default configuration applied during boot,
 * refer to recovery.h -> DEFAULT_OPTIONS.
 *
 * This task defines the local timer, which informs its
 * users of time elapsed since last call for each user.
 * This timer is independent of the ThreadX timer (above),
 * and relies on HAL tick that is incremented every 1 ms.
 */

#include "platform.h"
#include "evaluation.h"
#include "recovery.h"
#include "kalman.h"
#include "dma.h"

TX_QUEUE shared;
TX_THREAD recovery_task;
ULONG recovery_stack[RECV_STACK_ULONG];

/// Last recorded time for each timer user.
volatile fu32 local_time[Time_Users] = {0};

/// Run time configuration mask.
atomic_uint_fast32_t config = DEFAULT_OPTIONS;

/// Defined and used by Ascent filter.
extern volatile fu8 renorm_step_mask;


/* ------ Local definitions ------ */

#define RECVQ_SIZE 64
static tx_align enum message recvq_pool[RECVQ_SIZE] = {0};

static TX_TIMER ep_timeout;

static volatile fu16 failures = 0;

static fu16 to_abort  = TO_ABORT;
static fu16 to_reinit = TO_REINIT;

static fu16 gps_delay_count = 0;
static fu16 gps_malform_count = 0;


/* ------ Recovery logic ------ */


/// Synchronously initializes each sensor.
static inline void try_reinit_sensors(void)
{
  enum sensor faulty = Sensors;

  log_msg("FC:RECV: trying to reinit sensors", 35);

  __disable_irq();

  if (init_baro(NULL) != HAL_OK) {
    faulty += (Sensor_Baro + 1);
  }

  if (init_gyro(NULL) != HAL_OK) {
    faulty += (Sensor_Gyro + 1);
  }

  if (init_accl(NULL) != HAL_OK) {
    faulty += (Sensor_Accl + 2);
  }

  __enable_irq();
  
  if (faulty == Sensors) {
    log_msg("FC:RECV: reinit OK", 20);
  } else {
    log_err("FC:RECV: reinit failed (%d)", faulty);
  }
}


/// Reset Distribution and Evaluation tasks. Grants Telemetry
/// task full scheduling time. If launch signal is sent after
/// abortion, the FC will attempt to resume normal opertaion.
static inline void auto_abort(void)
{
  tx_thread_reset(&evaluation_task);
  tx_thread_reset(&distribution_task);

  gps_delay_count = 0;
  gps_malform_count = 0;

  if (config & option(Lost_GroundStation))
  {
    /* Nowhere to expect commands from! Reinitialize. */
    to_abort = TO_ABORT * 10;

    enum message cmd = fc_mask(Launch_Signal);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }
  else
  {
    log_msg("FC:RECV: issued automatic abortion. "
            "Waiting for commands or launch signal.", 75);

    config |= option(In_Aborted_State);
  }
}


/// Signals other tasks to not fetch or use GPS data.
/// Reinitializes barometer with larger oversampling rates.
static inline void barometer_fallback(void)
{
  config &= ~option(GPS_Available);

  struct baro_config precise = {
    .osr_t = BARO_OSR_X1,
    .osr_p = BARO_OSR_X8,
    .odr = BARO_DEFAULT_ODR_SEL,
    .iir_coef = BARO_IIR_COEF_15,
  };

  /* This is the place to perform sensitive initializations,
   * because Recovery cannot be preempted by other tasks. */
  if (init_baro(&precise) == HAL_OK)
  {
    log_msg("FC:RECV: lost GPS, relying on barometer", 40);
  }
}


/// Process general command from either endpoint.
static inline void process_action(enum message cmd)
{
  UINT eval_old_pt;

  switch (cmd) {
    case Deploy_Parachute:
      co2_high(&config);
      return;

    case Expand_Parachute:
      if (config & option(Parachute_Deployed)) {
        reef_high(&config);
      } else {
        log_err("FC:RECV: you have to deploy parachute first");
      }
      return;

    case Reinit_Sensors:
      try_reinit_sensors();
      return;

    case Launch_Signal:
      if (gps_delay_count || gps_malform_count)
      {
        log_err("FC:RECV: had %u GPS delays and %u malformed packets"
                " during pre-launch stage. Counters are now reset.",
                gps_delay_count, gps_malform_count);

        gps_delay_count = 0;
        gps_malform_count = 0;
      }

      if (config & option(In_Aborted_State))
      {
        tx_thread_resume(&distribution_task);
        config &= ~option(In_Aborted_State);
      }

      /* Start Evaluation and begin rocket launch chain. */
      tx_thread_resume(&evaluation_task);
      return;

    case Evaluation_Relax:
      /* Allow preemption of Evaluation task inside KF. */
      config &= ~option(Eval_Focus_Flag);
      tx_thread_preemption_change(&evaluation_task,
                                  EVAL_PRIORITY,
                                  &eval_old_pt);
      return;

    case Evaluation_Focus:
      /* Restrict preemption of Evaluation task inside KF. */
      config |= option(Eval_Focus_Flag);
      tx_thread_preemption_change(&evaluation_task,
                                  EVAL_PREEMPT_THRESHOLD,
                                  &eval_old_pt);
      return;

    case Evaluation_Abort:
      config |= option(Eval_Abort_Flag);
      tx_thread_reset(&evaluation_task);
      return;

    case Reinit_Barometer:
      barometer_fallback();
      return;

    default: break;
  }
}


/// Apply or revoke passed runtime configuration option.
static inline void process_config(enum message code)
{
  if (code & KF_Operation_Mode)
  {
    renorm_step_mask = code & ~KF_Operation_Mode;
  }
  else if (code & Abortion_Thresholds)
  {
    to_abort = code & ~Abortion_Thresholds;
  }
  else if (code & Reinit_Thresholds)
  {
    to_reinit = code & ~Reinit_Thresholds;
  }
  else if (code & Revoke_Option)
  {
    config &= ~(code & ~Revoke_Option);
  }
  /* If nothing else matched, must have exactly 1 bit set
   * (i.e., a power of 2) to be a valid config option. */
  else if (!(code & (code - 1)))
  {
    config |= code;
  }
  else
  {
    log_err("FC:RECV: invalid config option: %u", (unsigned)code);
  }
}


/// Checks whether raw data report is OK.
static inline void process_report(enum message code)
{
  if (code != Sensor_Measm_Code)
  {
    ++failures;
    log_err("FC:RECV: bad sensor reading (%u)", (unsigned)code);

    if (failures >= to_abort) {
      auto_abort();
    } else if (failures >= to_reinit) {
      try_reinit_sensors();
    }
  }
  else if (config & option(Reset_Failures))
  {
    failures = 0;
  }
}


/// Handles delayed or malformed GPS data report.
static inline void process_gps_code(enum message code)
{
  switch (code) {
    case GPS_Delayed:
      ++gps_delay_count;

      if (config & option(Launch_Triggered)) {
        log_err("FC:RECV: delayed GPS packet (%u)", gps_delay_count);
        return;
      }

      if (gps_delay_count >= MAX_GPS_DELAYS) {
        barometer_fallback();
        return;
      }
      break;

    case GPS_Malformed:
      ++gps_malform_count;

      if (config & option(Launch_Triggered)) {
        log_err("FC:RECV: malformed GPS packet (%u)", gps_malform_count);
        return;
      }

      if (gps_malform_count >= GPS_MAX_MALFORMED) {
        barometer_fallback();
        return;
      }
      break;

    default: break;
  }
}


/// Decodes universal FC message from an endpoint {FC, RF, GND}.
static inline void decode_message(enum message msg)
{
  if (msg & GroundStation_Heartbeat)
  {
    timer_update(HeartbeatGND);
  }
  else if (msg & Runtime_Configuration)
  {
    process_config(option(fc_unmask(msg)));
  }
  else if (msg & GPS_Data_Code)
  {
    process_gps_code(fc_unmask(msg));
  }
  else if (msg & Actionable_Decrees)
  {
    process_action(fc_unmask(msg));
  }
  else if (msg & Spurious_Confirmations)
  {
    msg &= ~(FC_Identifier | Spurious_Confirmations);
    log_err("FC:RECV: unconfirmed transition: %u", (unsigned)msg);
  }
  else if (fc_unmask(msg))
  {
    timer_update(HeartbeatFC);
    process_report(fc_unmask(msg));
  }
}


/// Suspend on blocking queue until a new message arrives.
void recovery_entry(ULONG input)
{
  (void) input;

  for (enum fc_timer k = 0; k < Time_Users; ++k) {
    timer_update(k);
  }

  tx_timer_activate(&ep_timeout);

  task_loop (DO_NOT_EXIT)
  {
    enum message msg;

    /* Thread suspension */
    UINT st = tx_queue_receive(&shared, &msg, TX_WAIT_FOREVER);

    if (st != TX_SUCCESS) {
      continue;
    }

    decode_message(msg);
  }
}

/*
 * This routine is called from ThreadX interrupt
 * every 0.5 seconds. It monitors timeout for the
 * endpoints flight computer depends on {FC, RF, GND},
 * and if timeout has occured, signals tasks to not
 * depend on the endpoint which has timed out. In case
 * of the FC itself, it tries to reinitialize vital
 * tasks 3 times and them aborts.
 *
 * This timer is also responsible for pulling CO2 and
 * REEF pins back low after either has been asserted.
 */
static void fc_timer_routine(ULONG id)
{
  (void) id;

  static fu8 restart_count = 0;

  if (config & option(CO2_Asserted) &&
      timer_fetch(AssertCO2) >= CO2_ASSERT_INTERVAL)
  {
    co2_low(&config);
    config &= ~option(CO2_Asserted);
  }

  if (config & option(REEF_Asserted) &&
      timer_fetch(AssertREEF) >= REEF_ASSERT_INTERVAL)
  {
    reef_low(&config);
    config &= ~option(REEF_Asserted);
  }

  if (config & option(In_Aborted_State))
  {
    /* If aborted, no task is sending heartbeat */
    timer_update(HeartbeatFC);
  }
  else if (timer_fetch(HeartbeatFC) > FC_TIMEOUT)
  {
    if (++restart_count >= MAX_RESTARTS) {
      auto_abort();
      return;
    }

    timer_update(HeartbeatFC);

    if (config & option(Launch_Triggered))
    {
      tx_thread_reset(&evaluation_task);
      tx_thread_resume(&evaluation_task);
    }
    else
    {
      tx_thread_reset(&distribution_task);
      tx_thread_resume(&distribution_task);
    }
  }

  
  if (timer_fetch(HeartbeatGND) > GND_TIMEOUT)
  {
#ifdef TELEMETRY_ENABLED
    config |= option(Lost_GroundStation);
    config |= option(Monitor_Altitude);
    config |= option(Reset_Failures);
    renorm_step_mask = RENORM_STEP;
    failures = 0;

#else /* FOR TESTS ONLY */
    static fu8 test_launched = 0;

    if (!test_launched) {
      tx_thread_resume(&evaluation_task);
      test_launched = 1;
    }

#endif // TELEMETRY_ENABLED
  }

  if (config & option(GPS_Available))
  {
    if (timer_fetch(HeartbeatRF) > GPS_DELAY_MS)
    {
      enum message cmd = fc_mask(GPS_Delayed);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }
  }
  else
  {
    if (timer_fetch(HeartbeatRF) < TX_TIMER_TICKS)
    {
      /* GPS became alive, announce */
      config |= option(GPS_Available);
    }
  }
}


/// Creates a non-preemptive Recovery Task with defined parameters.
void create_recovery_task(void)
{
  UINT st = tx_thread_create(&recovery_task,
                             "Recovery Task",
                             recovery_entry,
                             RECV_INPUT,
                             recovery_stack,
                             RECV_STACK_BYTES,
                             RECV_PRIORITY,
                             /* No preemption threshold */
                             RECV_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to create task (%u)", st);
  }

  st = tx_queue_create(&shared, "RECVQ", 1, &recvq_pool,
                                     sizeof recvq_pool);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to create queue (%u)", st);
  }

  st = tx_timer_create(&ep_timeout, "Endpoints timeout",
                       fc_timer_routine, 0, TX_TIMER_INITIAL,
                       TX_TIMER_TICKS, TX_NO_ACTIVATE);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to create timer (%u)", st);
  }
}