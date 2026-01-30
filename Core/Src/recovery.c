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

#include <stdint.h>
#include <stdatomic.h>

#include "platform.h"
#include "evaluation.h"
#include "recovery.h"
#include "dma.h"

TX_QUEUE shared;
TX_THREAD recovery_task;
ULONG recovery_stack[RECOVERY_STACK_ULONG];

/// Last recorded time for each timer user.
uint32_t local_time[Time_Users] = {0};

/// Run time configuration mask
atomic_uint_least32_t config = 0;


/* ------ User run time configuration ------ */

/// Run time config applied on boot.
/// Call this function once (before creating tasks).
/// Users are welcome to edit this function.
void user_runtime_config()
{
  uint_least32_t mode = 0;

  /* TODO discuss defaults */
  mode |= CONSECUTIVE_SAMP;

  atomic_store_explicit(&config, mode, memory_order_release);
}


/* ------ Local definitions ------ */

static TX_TIMER endpoints;
static TX_SEMAPHORE unread;
static uint_fast8_t failures = 0;

/// Queue of 32 ints that begins in the middle of SRAM1
#define QADDR (VOID *)0x20010000
#define QSIZE 128


/* ------ Recovery logic ------ */

/// Wake up recovery loop.
static void queue_handler(TX_QUEUE *q)
{
  if (q != &shared) return;
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

/// Set cautionary and emergency flags on timeout
static inline void
handle_timeout(enum fc_timer endpoint)
{
  uint_least32_t mode = 0;

  switch (endpoint)
  {
    case Recovery_FC:
      /* TODO to be discussed */
      auto_abort();
      break;

    case Recovery_GND:
      mode |= FORCE_ALT_CHECKS;
      mode |= ACCUMULATE_FAILS;
      mode &= ~CONSECUTIVE_SAMP;
      break;

    default: break;
  }

  atomic_fetch_or_explicit(&config, mode, memory_order_release);
}

// Process general command from either endpoint.
static inline void
process_action(enum command cmd, ULONG *conf)
{
  switch (cmd) {
    case FIRE_PYRO:
      co2_high();
      *conf |= SAFE_EXPAND_REEF;
      return;

    case FIRE_REEF:
      if (*conf & SAFE_EXPAND_REEF)
        reef_high();
      return;

    case RECOVER:
      try_reinit_sensors();
      *conf |= REINIT_ATTEMPTED;
      return;

    default: break;
  }
}

/// Checks whether raw data report is OK.
static inline void
process_raw_data_code(enum command code, ULONG *conf)
{
  if (code != RAW_DATA)
  {
    ++failures;
    log_err("FC:RECV: bad sensor reading (%d)", code);
  }
  else if (!(*conf & ACCUMULATE_FAILS))
  {
    /* RAW_DATA is sent when raw data looks good */
    failures = 0;
  }
}

/// Decodes message from the Flight Computer.
static inline void
decode_fc(enum command cmd, ULONG *conf)
{
  timer_update(Recovery_FC);

  if (cmd & ACTION)
  {
    process_action(cmd, conf);
  }
  else if (cmd & DATA_EVALUATION)
  {
    /* Report. Does not need processing function */
    log_err("FC:RECV: received warning (%d)", cmd);
  }
  else /* if we have raw data report */
  {
    process_raw_data_code(cmd, conf);
  }
}

/// Decodes message from the Ground Station.
static inline void
decode_gnd(enum command cmd, ULONG *conf)
{
  timer_update(Recovery_GND); // <---.
                              //     |
  if (cmd & SYNC)             //     |
  {                           //     |
    return; // Successful sync ------`
  }
  else if (cmd & ACTION)
  {
    process_action(cmd, conf);
  }
  else /* Programmer's mistake, codes < ACTION are FC-only */
  {
    log_err("FC:RECV: undefined command or code (%u)", cmd);
  }
}

/// Suspend on semaphore until a new message arrives.
/// Does not waste MCU cycles if queue if empty.
/// See page 68 of Azure RTOS ThreadX User Guide, Feb 2020.
///
/// Recovery task should never return.
void recovery_entry(ULONG conf)
{
  conf = 0;

  while (SEDS_ARE_COOL) {
    enum command cmd;

    /* Thread suspension */
    tx_semaphore_get(&unread, TX_WAIT_FOREVER);
    
    if (tx_queue_receive(&shared, &cmd,
        TX_WAIT_FOREVER) != TX_SUCCESS || !cmd)
    {
      continue;
    }

    if (cmd & FC_MASK) {
      /* Clear endpoint bit before decoding */
      decode_fc(cmd & ~FC_MASK, &conf);
    } else {
      decode_gnd(cmd, &conf);
    }

    if (failures >= FAILS_TO_ABORT) {
      auto_abort();
    } else if (failures >= FAILS_TO_REINIT) {
      try_reinit_sensors();
    }
  }
}

/// Check if an endpoint {FC, GND} is absent for
/// compile-defined time, and invoke handler appropriately.
static void check_endpoints(ULONG id)
{
  (void)id;

  if (timer_fetch(Recovery_FC) > FC_TIMEOUT_MS)
  {
    handle_timeout(Recovery_FC);
  }
  if (timer_fetch(Recovery_GND) > GND_TIMEOUT_MS)
  {
    handle_timeout(Recovery_GND);
  }
}

/// Creates a non-preemptive Recovery Task
/// with defined parameters. Called manually.
///
/// Stack size and priority are configurable in FC-Threads.h.
void create_recovery_task(void)
{
  UINT st = tx_thread_create(&recovery_task,
                             "Recovery Task",
                             recovery_entry,
                             RECOVERY_INPUT,
                             recovery_stack,
                             RECOVERY_STACK_BYTES,
                             RECOVERY_PRIORITY,
                             /* No preemption */
                             RECOVERY_PRIORITY,
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