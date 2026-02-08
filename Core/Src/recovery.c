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
 * The function user_runtime_config is provided,
 * where users can specify the default configuration.
 * The list of options can be found in recovery.h ->
 * -> "Run time configuration flags".
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

/// Run time configuration mask
/// Recovery task cannot be preempted and thus
/// can access this config non-atomically
atomic_uint_fast32_t config = DEFAULT_OPTIONS;


/* ------ Local definitions ------ */

static TX_TIMER endpoints;
static TX_SEMAPHORE unread;

static fu8 failures = 0;

static fu8 to_abort  = TO_ABORT;
static fu8 to_reinit = TO_REINIT;

/// Queue of 32 uints that begins in the middle of SRAM1
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


/// Scenario when Flight Computer itself decides to abort.
static inline void auto_abort()
{
  // TODO to be discussed
}


/// Process general command from either endpoint.
static inline void
process_action(enum command cmd)
{
  UINT eval_old_pt;

  switch (cmd) {
    case FIRE_PYRO:
      co2_high();
      return;

    case FIRE_REEF:
      if (config & SAFE_EXPAND_REEF) {
        reef_high();
      }
      return;

    case RECOVER:
      try_reinit_sensors();
      return;

    case START:
      /* Set Evaluation task eligible for scheduling. 
       * This begins rocket launch chain. */
      tx_thread_resume(&evaluation_task);
      return;

    case EVAL_RELAX:
      /* Allow preemption of Evaluation
       * task while inside Kalman Filter. */
      config &= ~EVAL_PREEMPT_OFF;
      tx_thread_preemption_change(&evaluation_task,
                                  EVAL_PRIORITY,
                                  &eval_old_pt);
      return;

    case EVAL_FOCUS:
      /* Rectrict preemption of Evaluation
       * task while inside Kalman Filter. */
      config |= EVAL_PREEMPT_OFF;
      tx_thread_preemption_change(&evaluation_task,
                                  EVAL_PREEMPT_THRESHOLD,
                                  &eval_old_pt);
      return;

    case EVAL_ABORT:
      /* Do not use ThreadX API to terminate
       * thread to avoid doing so when it is
       * inside HAL or ThreadX call. */
      config |= ABORT_EVALUATION;
      return;

    case ALT_CHECKS:
      config |= FORCE_ALT_CHECKS;
      return;

    case ACCUM_FAILS:
      config |= ACCUM_FAILS;
      return;

    default: break;
  }
}


/// Checks whether raw data report is OK.
static inline void
process_raw_data_code(enum command code)
{
  if (code != RAW_DATA)
  {
    ++failures;
    log_err("FC:RECV: bad sensor reading (%u)", (unsigned)code);

    if (failures >= to_abort) {
      auto_abort();
    } else if (failures >= to_reinit) {
      try_reinit_sensors();
    }
  }
  else if (config & RESET_FAILURES)
  {
    /* RAW_DATA is sent when raw data looks good */
    failures = 0;
  }
}


/// Handles delayed GPS data report.
static inline void
process_gps_code(enum command code)
{
  static fu8 delay_count = 0;
  static fu8 malformed_count = 0;

  switch (code) {
    case GPS_DELAY:
      if (++delay_count >= MAX_GPS_DELAYS) {
        goto force_ukf;
      }
      return;

    case GPS_MALFORMED:
      if (++malformed_count >= GPS_MAX_MALFORMED) {
        goto force_ukf;
      }
      return;

    default: return;
  }

force_ukf:
  config |= USE_ASCENT;

  if (!unscented) {
    initialize_ascent();
    unscented = 1;
  }
}


/// Decodes commands, message, or code from
/// the Flight Computer and the Ground Station.
static inline void
decode(enum command cmd)
{
  timer_update(cmd & FC_MASK ? HeartbeatFC : HeartbeatGND);
                  //                       ^
  if (cmd & SYNC) //                       |
  {         //                             |
    return; // Successful heartbeat -------`
  }
  else if (cmd & GPS_DELIVERY)
  {
    process_gps_code(cmd & ~FC_MASK);
  }
  else if (cmd & KF_OP_MODE)
  {
    /* Clear flag; what is left is the new step */
    config |= (cmd & ~KF_OP_MODE);
  }
  else if (cmd & AUTO_REINIT_BOUNDS)
  {
    /* Clear flag; what is left is the new bound */
    to_reinit = cmd & ~AUTO_REINIT_BOUNDS;
  }
  else if (cmd & AUTO_ABORT_BOUNDS)
  {
    to_abort = cmd & ~AUTO_ABORT_BOUNDS;
  }
  else if (cmd & ACTION)
  {
    process_action(cmd & ~FC_MASK);
  }
  else if (cmd & DATA_EVALUATION)
  {
    /* Data evaluation reports unconfirmed states.
     * For bookkeeping. */
    log_err("FC:RECV: received warning (%u)", (unsigned)cmd);
  }
  else if (cmd & FC_MASK)
  {
    /* Likely raw data report */
    process_raw_data_code(cmd & ~FC_MASK);
  }
}


/// Suspend on semaphore until a new message arrives.
/// Does not waste MCU cycles if queue if empty.
/// See page 68 of Azure RTOS ThreadX User Guide, Feb 2020.
///
/// Recovery task should never return.
void recovery_entry(ULONG input)
{
  (void) input;

  /* This task is created and ran first, and should
   * prevent accidental timeouts due to slow SW init. */
  timer_update(HeartbeatFC);
  timer_update(HeartbeatGND);

  task_loop (DO_NOT_EXIT)
  {
    enum command cmd;

    /* Thread suspension */
    tx_semaphore_get(&unread, TX_WAIT_FOREVER);
    
    if (tx_queue_receive(&shared, &cmd,
        TX_WAIT_FOREVER) != TX_SUCCESS || !cmd)
    {
      continue;
    }

    decode(cmd);
  }
}


/// Check if an endpoint {FC, GND} is absent for
/// compile-defined time, and invoke handler appropriately.
/// Runs in ThreadX interrupt context.
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

    /* Whether we have launched determines
     * which task sends heartbeat signals.
     * Accordingly, restart this task. */
    if (config & ENTER_DIST_CYCLE)
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
    /* Contact lost with Ground Station.
     * Set options for most precision and do not
     * accumulate failures. In fact, reset them! */
    config |= FORCE_ALT_CHECKS;
    config |= RENORM_QUATERN_1;
    config |= RESET_FAILURES;
    failures = 0;

#else
    static fu8 test_launched = 0;

    if (!test_launched) {
      /* During testing, launch on first GND timeout 
       * (since, without telemetry, there is no GND). */
      tx_thread_resume(&evaluation_task);
      test_launched = 1;
    }

#endif // TELEMETRY_ENABLED
  }

#ifdef GPS_AVAILABLE
  if (timer_fetch(HeartbeatRF) > RF_TIMEOUT_MS)
  {
    config |= USE_ASCENT;
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