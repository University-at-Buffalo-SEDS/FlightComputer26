/*
 * Recovery Task
 *
 * This task suspends on a blocking message queue
 * and when resumed, drains the message queue. The
 & messages are then interpreted depending on the
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
fu32 local_time[Time_Users] = {0};

/// Run time configuration mask.
atomic_uint_fast32_t config = DEFAULT_OPTIONS;

/// Defined and used by Ascent filter.
extern fu8 renorm_step_mask;


/* ------ Local definitions ------ */

static TX_TIMER ep_timeout;

static fu16 failures = 0;
static fu16 to_abort  = TO_ABORT;
static fu16 to_reinit = TO_REINIT;

static fu16 gps_delay_count = 0;
static fu16 gps_malform_count = 0;

#define QADDR (VOID *)0x20010000
#define QSIZE 128


/* ------ Recovery logic ------ */


/// Synchronously initializes each sensor.
static inline void try_reinit_sensors(void)
{
  enum device faulty = Sensors;

  log_msg("FC:RECV: trying to reinit sensors", 35);

  __disable_irq();

  if (init_baro() != HAL_OK) {
    faulty += (Sensor_Baro + 1);
  }

  if (init_gyro() != HAL_OK) {
    faulty += (Sensor_Gyro + 1);
  }

  if (init_accl() != HAL_OK) {
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
  config |= static_option(In_Aborted_State);

  tx_thread_reset(&evaluation_task);
  tx_thread_reset(&distribution_task);

  gps_delay_count = 0;
  gps_malform_count = 0;

  log_msg("FC:RECV: issued automatic abortion. "
          "Waiting for commands or launch signal.", 75);

  if (config & static_option(Lost_GroundStation))
  {
    /* Nowhere to expect commands from! Reinitialize. */
    to_abort = TO_ABORT * 10;

    enum message cmd = fc_mask(Launch_Signal);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }
}


/// Process general command from either endpoint.
static inline void process_action(enum message cmd)
{
  UINT eval_old_pt;

  switch (cmd) {
    case Deploy_Parachute:
      co2_high();
      return;

    case Expand_Parachute:
      if (config & static_option(Parachute_Deployed)) {
        reef_high();
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

      if (config & static_option(In_Aborted_State))
      {
        tx_thread_resume(&distribution_task);
        config &= ~static_option(In_Aborted_State);
      }

      /* Start Evaluation and begin rocket launch chain. */
      tx_thread_resume(&evaluation_task);
      return;

    case Evaluation_Relax:
      /* Allow preemption of Evaluation task inside KF. */
      config &= ~static_option(Eval_Focus_Flag);
      tx_thread_preemption_change(&evaluation_task,
                                  EVAL_PRIORITY,
                                  &eval_old_pt);
      return;

    case Evaluation_Focus:
      /* Restrict preemption of Evaluation task inside KF. */
      config |= static_option(Eval_Focus_Flag);
      tx_thread_preemption_change(&evaluation_task,
                                  EVAL_PREEMPT_THRESHOLD,
                                  &eval_old_pt);
      return;

    case Evaluation_Abort:
      config |= static_option(Eval_Abort_Flag);
      tx_thread_reset(&evaluation_task);
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
  else
  {
    config |= code;
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
  else if (config & static_option(Reset_Failures))
  {
    failures = 0;
  }
}


/// Signals other tasks to use Ascent filter, if not already.
static inline void prohibit_descent_kf(void)
{
  config &= ~static_option(Descent_KF_Feasible);
  log_msg("FC:RECV: blocked Descent KF (problems with GPS)", 48);

  if (!(config & static_option(Using_Ascent_KF))) {
    initialize_ascent();
    config |= static_option(Using_Ascent_KF);
  }
}


/// Handles delayed or malformed GPS data report.
static inline void process_gps_code(enum message code)
{
  switch (code) {
    case GPS_Delayed:
      ++gps_delay_count;

      if (config & static_option(Launch_Triggered)) {
        log_err("FC:RECV: delayed GPS packet (%u)", gps_delay_count);
        return;
      }

      if (gps_delay_count >= MAX_GPS_DELAYS) {
        prohibit_descent_kf();
        return;
      }
      break;

    case GPS_Malformed:
      ++gps_malform_count;

      if (config & static_option(Launch_Triggered)) {
        log_err("FC:RECV: malformed GPS packet (%u)", gps_malform_count);
        return;
      }

      if (gps_malform_count >= GPS_MAX_MALFORMED) {
        prohibit_descent_kf();
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
    process_config(static_option(fc_unmask(msg)));
  }
  else if (msg & GPS_Packet_Code)
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


/// Check if an endpoint {FC, GND, RF} is absent for
/// compile-defined time. If it is, act appropriately.
static void check_endpoints(ULONG id)
{
  (void) id;

  static fu8 restart_count = 0;

  if (config & static_option(In_Aborted_State))
  {
    /* If aborted, no task is sending heartbeat */
    timer_update(HeartbeatFC);
  }
  else if (timer_fetch(HeartbeatFC) > FC_TIMEOUT_MS)
  {
    if (++restart_count >= MAX_RESTARTS) {
      auto_abort();
      return;
    }

    timer_update(HeartbeatFC);

    if (config & static_option(Launch_Triggered))
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

  
  if (timer_fetch(HeartbeatGND) > GND_TIMEOUT_MS)
  {
#ifdef TELEMETRY_ENABLED
    config |= static_option(Lost_GroundStation);
    config |= static_option(Monitor_Altitude);
    config |= static_option(Reset_Failures);
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

#ifdef GPS_AVAILABLE
  if (timer_fetch(HeartbeatRF) > RF_TIMEOUT_MS)
  {
    config &= ~static_option(Descent_KF_Feasible);

    if (!(config & static_option(Using_Ascent_KF))) {
      initialize_ascent();
      config |= static_option(Using_Ascent_KF);
    }
  }

#endif // GPS_AVAILABLE
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

  st = tx_queue_create(&shared, "FC messages", 1, QADDR, QSIZE);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to create queue (%u)", st);
  }

  st = tx_timer_create(&ep_timeout, "Endpoints timeout",
                       check_endpoints, 0, TX_TIMER_INITIAL,
                       TX_TIMER_TICKS, TX_NO_ACTIVATE);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to create timer (%u)", st);
  }
}