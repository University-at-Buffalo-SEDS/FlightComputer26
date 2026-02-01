/*
 * Thread component of the Telemetry task.
 * Invokes CAN and sedsprintf_rs router
 * from the ThreadX thread context.
 */

#include "platform.h"
#include "telemetry.h"
#include "can_bus.h"


/// Stack + TCB for telemetry thread
TX_THREAD telemetry_thread;
ULONG telemetry_thread_stack[TLMT_STACK_ULONG];

/// Telemetry task entry.
void telemetry_thread_entry(ULONG initial_input)
{
  (void)initial_input;

  const char started_txt[] = "Telemetry thread starting";
  log_telemetry_synchronous(SEDS_DT_MESSAGE_DATA,
                            started_txt,
                            sizeof(started_txt),
                            1);

  task_loop (DO_NOT_EXIT)
  {
    can_bus_process_rx();
    process_all_queues_timeout(5);
    can_bus_process_rx();
  }
}

/// Creates a preemptive, non-cooperative telemetry thread
/// with defined parameters. This thread repeatedly drains
/// CAN bus message queue and invokes local instance of the
/// telemetry router to process incoming and outcoming messages.
void create_telemetry_thread(void)
{
  UINT status = tx_thread_create(&telemetry_thread,
                                  "Telemetry Task",
                                  telemetry_thread_entry,
                                  TLMT_INPUT,
                                  telemetry_thread_stack,
                                  TLMT_STACK_BYTES,
                                  /* No preemption threshold */
                                  TLMT_PRIORITY,
                                  TLMT_PRIORITY,
                                  TLMT_TIME_SLICE,
                                  TX_AUTO_START);

  if (status != TX_SUCCESS) {
    die("Failed to create telemetry thread: %u", status);
  }
}