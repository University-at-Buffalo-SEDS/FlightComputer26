// telemetry_thread.c

#include "platform.h"
#include "fctasks.h"
#include "can_bus.h"

TX_THREAD telemetry_thread;

void telemetry_thread_entry(ULONG initial_input)
{
  (void)initial_input;

  // Ensure router exists early (so we can send requests immediately)
  (void)init_telemetry_router();

  task_loop (DO_NOT_EXIT)
  {
    can_bus_process_rx();
    (void)telemetry_poll_discovery();
    (void)process_all_queues_timeout(50);
    (void)telemetry_poll_timesync();

    tx_thread_relinquish();
  }
}

UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool)
{
  CHAR *pointer;

  /* Allocate the stack for test  */
  if (tx_byte_allocate(byte_pool, (VOID**) &pointer,
                       TLMT_STACK_BYTES, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  UINT status = tx_thread_create(&telemetry_thread,
                                  "Telemetry Task",
                                  telemetry_thread_entry,
                                  TLMT_INPUT,
                                  pointer,
                                  TLMT_STACK_BYTES,
                                  /* No preemption threshold */
                                  TLMT_PRIORITY,
                                  TLMT_PRIORITY,
                                  TLMT_TIME_SLICE,
                                  TX_AUTO_START);

  return status;
}
