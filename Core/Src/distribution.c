/* Distribution Task
 *
 * Uses API of DMA, Telemetry, Predict, and Recovery modules
 * to serve as the Flight Computer's coordination module.
 *
 * Software model: this task should be trivial to
 * be able to demonstrate routing across rocket modules.
 * Synchronization and task specifics should be handled
 * with the respective APIs themselves.
 */

#include "platform.h"
#include "predict.h"
#include "recovery.h"

TX_THREAD distribution_task;
ULONG distribution_stack[SENSOR_STACK_ULONG];


/* ------ Distribution Task ------ */

/// An overview of raw data path inside rocket.
void distribution_entry(ULONG input)
{
  (void)input;
  timer_init();
  
  struct measurement payload = {0};

  while (SEDS_ARE_COOL)
  {
    if (!dma_try_fetch(&payload))
    {
      tx_thread_sleep(SENSOR_SLEEP);
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
}

/// Creates a non-preemptive Distribution Task
/// with defined parameters. Called manually.
/// Provides its entry point with a throwaway input.
///
/// Stack size and priority are configurable in FC-Threads.h.
void create_distribution_task(void)
{
  UINT st = tx_thread_create(&distribution_task,
                             "Distribution Task",
                             distribution_entry,
                             SENSOR_INPUT,
                             distribution_stack,
                             SENSOR_STACK_BYTES,
                             SENSOR_PRIORITY,
                             /* No preemption */
                             SENSOR_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("Failed to create Distribution Task: %u", (unsigned)st);
  }
}