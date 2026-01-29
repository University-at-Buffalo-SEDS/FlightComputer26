/* Sensor task
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

TX_THREAD sensor_task;
ULONG sensor_stack[SENSOR_STACK_ULONG];


/* ------ Sensor Task ------ */

/// An overview of raw data path inside rocket.
void sensor_entry(ULONG input)
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
  }
}

/// Creates a non-preemptive sensor task
/// with defined parameters. Called manually.
/// Provides its entry point with a throwaway input.
///
/// Stack size and priority are configurable in FC-Threads.h.
void create_sensor_task(void)
{
  UINT st = tx_thread_create(&sensor_task,
                             "Sensor Task",
                             sensor_entry,
                             SENSOR_INPUT,
                             sensor_stack,
                             SENSOR_STACK_BYTES,
                             SENSOR_PRIORITY,
                             /* No preemption */
                             SENSOR_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("Failed to create Sensor Task: %u", (unsigned)st);
  }
}