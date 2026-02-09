/*
 * Recovery Task
 *
 * This task suspends on a counting semaphore
 * and when resumed, drains the message queue.
 * The messages are then interpreted depending
 * on the MSB designating the endpoint - FC or GND.
 * When a message appears in the queue, a callback
 * is invoked that increments the value in semaphore.
 *
 * This task also includes a timer callback that
 * checks for timeout of both FC and GND. If timeout
 * occurs (i.e., this task did not receive a signal)
 * for 4 seconds, a handler is invoked that changes
 * global runtime configuration, which other tasks
 * read and locally update on every iteration. The
 * ThreadX timer is set to expire every second.
 *
 * To change default configuration applied during boot,
 * refer to recovery.h -> DEFAULT_OPTIONS.
 *
 * You can send a command to recovery like this
 * (assume firing pyro from the ground station):
 *
 * enum command cmd = FIRE_PYRO;
 * tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
 *
 * Wait option depends on whether you want to drop
 * the value if queue is full (unlikely). 
 *
 * NOTE: if you are sending a message from FC,
 * mask it with FC_MSG(_variant_). Otherwise - UB!
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

static TX_TIMER endpoints;
static TX_SEMAPHORE unread;

static fu8 failures = 0;

static fu8 to_abort  = TO_ABORT;
static fu8 to_reinit = TO_REINIT;

#define QADDR (VOID *)0x20010000
#define QSIZE 128


/* ------ Recovery logic ------ */

/// Wake up recovery loop.
static void queue_handler(TX_QUEUE *q)
{
  if (q != &shared) {
    return;
  }
  tx_semaphore_put(&unread);
}


/// Synchronously initializes each sensor.
static inline void try_reinit_sensors()
{
  enum device faulty = DEVICES;

  log_msg("FC:RECV: trying to reinit sensors", 35);

  __disable_irq();

  if (init_baro() != HAL_OK) {
    faulty *= BAROMETER;
  }
  if (init_gyro() != HAL_OK) {
    faulty -= GYROSCOPE;
  }
  if (init_accel() != HAL_OK) {
    faulty -= ACCELEROMETER;
  }

  __enable_irq();
  
  if (faulty == DEVICES) {
    log_msg("FC:RECV: reinit OK", 20);
  } else {
    log_err("FC:RECV: reinit failed (%d)", faulty);
  }
}


/// FC deemed the required minimum of sensors unreliable.
static inline void auto_abort()
{
  // TODO
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
      }
      return;

    case Reinit_Sensors:
      try_reinit_sensors();
      return;

    case Launch_Signal:
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
  if (code & KF_Operation_Mode) {
    renorm_step_mask = code & ~KF_Operation_Mode;
  } else if (code & Abortion_Thresholds) {
    to_abort = code & ~Abortion_Thresholds;
  } else if (code & Reinit_Thresholds) {
    to_reinit = code & ~Reinit_Thresholds;
  } else if (code & Revoke_Option) {
    config &= ~(code & ~Revoke_Option);
  } else {
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


/// Handles delayed or malformed GPS data report.
static inline void process_gps_code(enum message code)
{
  static fu8 delay_count = 0;
  static fu8 malformed_count = 0;

  switch (code) {
    case GPS_Delayed:
      if (++delay_count >= MAX_GPS_DELAYS) {
        goto force_ukf;
      }
      return;

    case GPS_Malformed:
      if (++malformed_count >= GPS_MAX_MALFORMED) {
        goto force_ukf;
      }
      return;

    default: return;
  }

force_ukf:
  config |= static_option(Prohibit_Descent_KF);

  if (!unscented) {
    initialize_ascent();
    unscented = 1;
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
    process_config(static_option(msg & ~FC_Identifier));
  }
  else if (msg & GPS_Packet_Code)
  {
    process_gps_code(msg & ~FC_Identifier);
  }
  else if (msg & Actionable_Decrees)
  {
    process_action(msg & ~FC_Identifier);
  }
  else if (msg & Spurious_Confirmations)
  {
    msg &= ~(FC_Identifier | Spurious_Confirmations);
    log_err("FC:RECV: unconfirmed transition: %u", (unsigned)msg);
  }
  else if (msg & FC_Identifier)
  {
    timer_update(HeartbeatFC);
    process_report(msg & ~FC_Identifier);
  }
}


/// Suspend on semaphore until a new message arrives.
/// Does not waste cycles if queue is empty.
void recovery_entry(ULONG input)
{
  (void) input;

  for (enum fc_timer k = 0; k < Time_Users; ++k) {
    timer_update(k);
  }

  task_loop (DO_NOT_EXIT)
  {
    enum message cmd;

    /* Thread suspension */
    tx_semaphore_get(&unread, TX_WAIT_FOREVER);
    
    if (tx_queue_receive(&shared, &cmd,
        TX_WAIT_FOREVER) != TX_SUCCESS || !cmd)
    {
      continue;
    }

    decode_message(cmd);
  }
}


/// Check if an endpoint {FC, GND, RF} is absent for
/// compile-defined time. If it is, act appropriately.
static void check_endpoints(ULONG id)
{
  (void) id;

  static fu8 restart_count = 0;

  if (timer_fetch(HeartbeatFC) > FC_TIMEOUT_MS)
  {
    if (++restart_count >= MAX_RESTARTS) {
      return auto_abort();
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
    config |= static_option(Prohibit_Descent_KF);
    if (!unscented) {
      initialize_ascent();
      unscented = 1;
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

  st = tx_queue_create(&shared, "Message queue", 1, QADDR, QSIZE);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to create queue (%u)", st);
  }

  st = tx_queue_send_notify(&shared, queue_handler);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to subscribe to queue (%u)", st);
  }

  st = tx_semaphore_create(&unread, "Messages in queue", 0);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to create semaphore (%u)", st);
  }

  st = tx_timer_create(&endpoints, "Endpoints timeout timer",
                       check_endpoints, 0, TX_TIMER_INITIAL,
                       TX_TIMER_TICKS, TX_AUTO_ACTIVATE);

  if (st != TX_SUCCESS) {
    log_die("FC:RECV: failed to create timer (%u)", st);
  }
}