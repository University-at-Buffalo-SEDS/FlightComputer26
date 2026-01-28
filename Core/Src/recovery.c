/*
 * Recovery and command execution.
 *
 * For example, to send FIRE_PYRO to recovery:
 *
 * #include "FC-Threads.h"
 * cmd_e command = FIRE_PYRO;
 * tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
 *
 * Wait option depends on whether you want to drop
 * the value if queue is full (unlikely). 
 */

#include <stdint.h>

#include "platform.h"
#include "recovery.h"

TX_QUEUE shared;
TX_THREAD recovery_task;
ULONG recovery_stack[RECOVERY_STACK_ULONG];

/// Last recorded time for each timer user.
uint32_t local_time[Time_Users] = {0};


/* ------ Local definitions ------ */

static TX_TIMER endpoints;
static TX_SEMAPHORE unread;
static uint_fast8_t failures = 0;

/// Queue of 32 ints that begins in the middle of SRAM1
#define QADDR (VOID *)0x20010000
#define QSIZE 128

/// Deployment masks
#define F_PYRO 0x01u
#define REINIT 0x10u

/// For sensor reinitialization.
typedef enum {
  RECOV_OK = 0,
  BAD_ACCL = 1,
  BAD_GYRO = 2,
  BAD_BARO = 3,
} recov_t;


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
  recov_t st = RECOV_OK;

  log_msg("Recovery: trying to reinit sensors", 35);

  __disable_irq();

  if (init_baro() != HAL_OK)
    st += BAD_BARO;

  if (init_gyro() != HAL_OK)
    st += BAD_GYRO;

  if (init_accel() != HAL_OK)
    st += BAD_ACCL;

  __enable_irq();
  
  if (st == RECOV_OK) {
    log_msg("Recovery: reinit OK", 20);
  } else {
    log_err("Recovery: reinit failed (%d)", st);
  }
}

static inline void abortion_due_failures()
{
  // TODO
}

static inline void handle_timeout(time_user_e endpoint)
{
  // TODO
}

// Process general command from either endpoint.
static inline void process_action(cmd_e cmd, ULONG *flag)
{
  switch (cmd) {
    case FIRE_PYRO:
      co2_high();
      *flag |= F_PYRO;
      return;

    case FIRE_REEF:
      if (*flag & F_PYRO)
        reef_high();
      return;

    case RECOVER:
      try_reinit_sensors();
      *flag |= REINIT;
      return;

    default: break;
  }
}

/// Checks whether raw data report is OK.
static inline void process_raw_data_code(cmd_e code)
{
  if (code != RAW_DATA) {
    ++failures;
    log_err("Recovery: bad sensor reading (%d)", code);
  }
#if ACCUMULATE_FAILURES <= 0
  else /* RAW_DATA is sent when raw data looks good */
  {
    failures = 0;
  }
#endif
}

/// Decodes message and calls appropriate handler.
static inline void decode(cmd_e cmd, ULONG *flag)
{
  if (cmd & FC_MASK) /* <--- FC section */
  {
    timer_update(Recovery_FC);

    cmd &= ~FC_MASK; // Clear the mask

    if (cmd & ACTION)
    {
      process_action(cmd, flag);
    }
    else if (cmd & DATA_EVALUATION)
    {
      /* Report: does not need processing function */
      log_err("Recovery: received warning (%d)", cmd);
    }
    else /* if we have raw data report */
    {
      process_raw_data_code(cmd);
    }
  }
  else /* <--- GND section */
  {
    timer_update(Recovery_GND);

    if (cmd & SYNC) {
      return; // Successful sync
    }
    else if (cmd & ACTION)
    {
      process_action(cmd, flag);
    }
    else /* Programmer's mistake, codes < ACTION are FC-only */
    {
      log_msg("Recovery: undefined command or code", 36);
    }
  }
}

/// Suspend on semaphore until a new message arrives.
/// Does not waste MCU cycles if queue if empty.
/// See page 68 of Azure RTOS ThreadX User Guide, Feb 2020.
void recovery_entry(ULONG flag)
{
  flag = 0;

  while (SEDS_ARE_COOL) {
    cmd_e cmd;

    /* Thread suspension */
    tx_semaphore_get(&unread, TX_WAIT_FOREVER);
    
    if (tx_queue_receive(&shared, &cmd,
        TX_WAIT_FOREVER) != TX_SUCCESS)
    {
      continue;
    }

    decode(cmd, &flag);

    if (failures >= FAILS_TO_ABORT) {
      abortion_due_failures();
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

/// Relies exclusively on TX primitives.
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
    log_die("Failed to create Recovery Task");
  }

  st = tx_queue_create(&shared, "Shared queue", 1, QADDR, QSIZE);
  if (st != TX_SUCCESS) {
    log_die("Failed to create shared queue");
  }

  st = tx_queue_send_notify(&shared, queue_handler);
  if (st != TX_SUCCESS) {
    log_die("Failed to subscribe to shared queue");
  }

  st = tx_semaphore_create(&unread, "Unread messages", 0);
  if (st != TX_SUCCESS) {
    log_die("Failed to create unread semaphore");
  }

  st = tx_timer_create(&endpoints, "Endpoints timerout timer",
                       check_endpoints, 0, TX_TIMER_INITIAL,
                       TX_TIMER_TICKS, TX_AUTO_ACTIVATE);
  if (st != TX_SUCCESS) {
    log_die("Failed to create timeout check timer");
  }
}