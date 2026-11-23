/* Core/Src/thread_comm.c */

#include "thread_comm.h"

static TX_MUTEX estimator_guard;
static bool initialized;

UINT thread_comm_init(void)
{
  if (initialized) return TX_SUCCESS;
  /* Priority inheritance: a recovery command waiting for the guard lifts the
   * evaluator to its priority until the pass ends, as before. */
  const UINT st = tx_mutex_create(&estimator_guard, "FC estimator", TX_INHERIT);
  if (st == TX_SUCCESS) initialized = true;
  return st;
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
