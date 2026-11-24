/* Core/Src/thread_comm.c */

#include "thread_comm.h"

static TX_MUTEX estimator_guard;
static TX_QUEUE commands;
static ULONG command_storage[THREAD_COMM_QUEUE_DEPTH * (sizeof(thread_comm_msg_t) / sizeof(ULONG))];
static bool initialized;

_Static_assert(sizeof(thread_comm_msg_t) == 2U * sizeof(ULONG), "two-word queue messages");

/* Diagnostic probe (FirmwareSimulator): the last network command accepted
 * into the queue, cleared when the control thread takes it. */
volatile uint32_t g_last_network_flight_command_msg
    __attribute__((used, externally_visible)) = UINT32_MAX;

UINT thread_comm_init(void)
{
  if (initialized) return TX_SUCCESS;
  /* Priority inheritance: a recovery command waiting for the guard lifts the
   * evaluator to its priority until the pass ends, as before. */
  UINT st = tx_mutex_create(&estimator_guard, "FC estimator", TX_INHERIT);
  if (st == TX_SUCCESS)
    st = tx_queue_create(&commands, "FC commands", sizeof(thread_comm_msg_t) / sizeof(ULONG),
                         command_storage, sizeof command_storage);
  if (st == TX_SUCCESS) initialized = true;
  return st;
}

UINT thread_comm_send(uint32_t cmd, ULONG wait_option)
{
  thread_comm_msg_t msg = { cmd, THREAD_COMM_LOCAL };
  return tx_queue_send(&commands, &msg, wait_option);
}

UINT thread_comm_send_network(uint32_t cmd)
{
  thread_comm_msg_t msg = { cmd, THREAD_COMM_NETWORK };
  /* Before the send: the receiver runs at a higher priority and may take the
   * message inside tx_queue_send. */
  g_last_network_flight_command_msg = cmd;
  const UINT st = tx_queue_send(&commands, &msg, TX_NO_WAIT);
  if (st != TX_SUCCESS && g_last_network_flight_command_msg == cmd)
    g_last_network_flight_command_msg = UINT32_MAX;
  return st;
}

UINT thread_comm_receive(thread_comm_msg_t *msg, ULONG wait_option)
{
  const UINT st = tx_queue_receive(&commands, msg, wait_option);
  if (st == TX_SUCCESS && msg->origin == THREAD_COMM_NETWORK)
    g_last_network_flight_command_msg = UINT32_MAX;
  return st;
}

void thread_comm_flush(void)
{
  (void)tx_queue_flush(&commands);
  g_last_network_flight_command_msg = UINT32_MAX;
}

bool thread_comm_receiver_waiting(const TX_THREAD *thread)
{
  return thread->tx_thread_state == TX_QUEUE_SUSP &&
         thread->tx_thread_suspend_control_block == &commands;
}

void thread_comm_estimator_enter(void)
{
  (void)tx_mutex_get(&estimator_guard, TX_WAIT_FOREVER);
}

void thread_comm_estimator_leave(void)
{
  (void)tx_mutex_put(&estimator_guard);
}

bool thread_comm_estimator_held(void)
{
  return initialized && estimator_guard.tx_mutex_owner == tx_thread_identify() &&
         estimator_guard.tx_mutex_ownership_count != 0U;
}

void thread_comm_estimator_sleep(ULONG ticks)
{
  /* Recovery commands may run while the evaluator sleeps, exactly as they
   * could when the sleep released the CPU under the old threshold. */
  ULONG depth = 0U;
  while (thread_comm_estimator_held()) { thread_comm_estimator_leave(); ++depth; }
  (void)tx_thread_sleep(ticks);
  while (depth--) thread_comm_estimator_enter();
}
