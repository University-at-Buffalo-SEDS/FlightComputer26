/*
 * Distribution Task
 *
 * Uses API of DMA, Telemetry, Predict, Recovery, and CAN
 * to serve as the Flight Computer coordination module.
 *
 * This task subscribes a callback to the CAN bus driver,
 * and calls its depletion function in the beginning of
 * each iteration. Before entering its main loop, this task
 * also sets time for the each FC user to the current tick.
 *
 * When fetching a pack of data (barometer, gyroscope, and
 * accelerometer readings) from DMA buffers, it is not
 * guaranteed that all three readings will be ready at once.
 * The underlying DMA fetch function is thus called repetedly
 * from the main loop until it (the function) notifies the
 * caller (this task) that the buffer is ready and it can
 * proceed. This is made to simplify synchronization within
 * the Interrupt Service Routine, where ThreadX API, and
 * therefore its wait/wake service, is unavailable.
 *
 * The data is then passed for preliminary compensation 
 * (provided by device drivers), and placed inside the
 * data evaluation buffer and telemetry (UART and SD) queues.
 *
 * Software model: this task should be trivial to
 * be able to demonstrate routing across rocket modules.
 * Synchronization and task specifics should be handled
 * with the respective APIs themselves.
 */

#include <stddef.h>
#include <stdint.h>

#include "platform.h"
#include "evaluation.h"
#include "recovery.h"

TX_THREAD distribution_task;
ULONG distribution_stack[DISTRIB_STACK_ULONG];


/* ------ CAN bus subscriber ------ */

/// CAN bus subscriber that puts messages into the FC queue. 
static void
on_can_frame(const uint8_t *data, size_t len, void *user)
{
  if (user != &distribution_task || !len || !(len & 3)) {
    /* Not the intended recepient,
     * or the payload length isn't divisible by 4 */
    return;
  }

  for (fu8 k = 0; k < len; k += sizeof(uint32_t))
  {
    uint32_t msg = U32(data[k], data[k+1], data[k+2], data[k+3]);
    UINT st = tx_queue_send(&shared, &msg, TX_NO_WAIT);

    if (st != TX_SUCCESS) {
      log_err("FC:DIST: failed to enqueue message %u (%u)", msg, st);
    }
  }
}


/* ------ Distribution Task ------ */

/// An overview of raw data path inside rocket.
void distribution_entry(ULONG input)
{
  (void)input;
  timer_init();
  
  struct measurement payload = {0};

  task_main_loop {
    /*can_bus_process_rx()*/

    if (!dma_try_fetch(&payload))
    {
      tx_thread_sleep(DISTRIB_SLEEP);
      continue;
    }

    compensate(&payload);
    evaluation_put(&payload);

    log_measurement(SEDS_DT_BAROMETER_DATA, &payload.baro);
    log_measurement(SEDS_DT_GYRO_DATA,      &payload.gyro);
    log_measurement(SEDS_DT_ACCEL_DATA,     &payload.accl);

    /// TODO: When merged with telemetry_handlers, log to SD.
  }

  /* Assert unreachable
   *(void)
   *can_bus_unsubscribe_rx(fc_distributor, &distribution_entry)*/
}

/// Creates a non-preemptive Distribution Task
/// with defined parameters. Called manually.
/// Provides its entry point with a throwaway input.
///
/// Stack size and priority are configurable in FC-Threads.h.
void create_distribution_task(void)
{
  UINT st = /*can_bus_subscribe_rx(on_can_frame, &distribution_entry)*/0;

  if (st != HAL_OK) {
    log_die("FC:DIST: failed to subscribe to CAN bus (%u)", st);
  }

  st = tx_thread_create(&distribution_task,
                        "Distribution Task",
                        distribution_entry,
                        DISTRIB_INPUT,
                        distribution_stack,
                        DISTRIB_STACK_BYTES,
                        DISTRIB_PRIORITY,
                        /* No preemption */
                        DISTRIB_PRIORITY,
                        TX_NO_TIME_SLICE,
                        TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("FC:DIST: failed to create task (%u)", st);
  }
}