/* Distribution Task
 *
 * Uses API of DMA, Telemetry, Predict, Recovery, and CAN
 * to serve as the Flight Computer coordination module.
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
fc_distributor(const uint8_t *data, size_t len, void *user)
{
  if (user != &distribution_task || !len || !(len & 3)) {
    /* Not the intended recepient,
     * or the payload length isn't divisible by 4 */
    return;
  }

  for (uint_fast8_t k = 0; k < len; k += sizeof(uint32_t))
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

  while (SEDS_ARE_COOL)
  {
    /*can_bus_process_rx()*/

    if (!dma_try_fetch(&payload))
    {
      tx_thread_sleep(DISTRIB_SLEEP);
      continue;
    }

    compensate(&payload);
    predict_put(&payload);

    log_measurement(SEDS_DT_BAROMETER_DATA, &payload);
    log_measurement(SEDS_DT_GYRO_DATA, &payload);
    log_measurement(SEDS_DT_ACCEL_DATA, &payload);

    /// TODO: When merged with telemetry_handlers, log to SD.
    /// and pass messages from fdcan to recovery queue.
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
  UINT st = /*can_bus_subscribe_rx(fc_distributor, &distribution_entry)*/0;

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