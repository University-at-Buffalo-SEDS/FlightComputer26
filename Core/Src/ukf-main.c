/*
 * Unscented Kalman Filter thread and base logic.
 */

#include "FC-Threads.h"
#include "platform.h"

TX_THREAD ukf_thread;
ULONG ukf_thread_stack[UKF_THREAD_STACK_SIZE];

void ukf_thread_entry(ULONG input)
{
  (void)input;

  log_msg_sync("UKF: starting thread", 21);

  for (;;)
  {
    // switch over value returned by get_rocket_state()
    // to select appropriate filter
  }
}

/// Creates a non-preemptive Unscented Kalman Filter (UKF)
/// thread with defined parameters. Called manually.
/// Provides its entry point with a throwaway input.
///
/// Stack size and priority are configurable in FC-Threads.h.
void create_ukf_thread(void)
{
  UINT st = tx_thread_create(&ukf_thread,
                             "UKF Thread",
                             ukf_thread_entry,
                             UKF_THREAD_INPUT,
                             ukf_thread_stack,
                             UKF_THREAD_STACK_SIZE,
                             UKF_THREAD_PRIORITY,
                             /* No preemption */
                             UKF_THREAD_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("Failed to create UKF thread: %u", (unsigned)st);
  }
}