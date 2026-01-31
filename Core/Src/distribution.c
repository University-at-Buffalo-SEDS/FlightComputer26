/*
 * Distribution Task
 *
 * Uses API of DMA, Telemetry, Predict, and Recovery
 * to serve as the Flight Computer coordination module.
 *
 * This task includes a packet handler that is comptime
 * subscribed to messages incoming to the FLIGHT_CONTROLLER
 * endpoint. This handler deposits messages into the Recovery
 * queue.
 *
 * Before entering the normal operation (flight) loop,
 * this task runs a pilot (demo) loop that fetches and
 * validates data but does not evaluate it. This is needed
 * in order to verify that sensors are functional and to inform
 * the Ground Station of any issues.
 *
 * The pilot loop is left and ignition is requested from the
 * Valve board upon receiving the 'ENTER_DIST_CYCLE' atomic
 * flag from the Evaluation Task, which sets this flag after
 * being (in turn) woken on a semaphore by the Recovery Task,
 * which sends it (in turn) after receiving the 'START' signal
 * on its queue, which (in turn) is deposited there by the
 * telemetry handler (below), being (in turn) invoked when
 * a local instance of the telemetry router receives a frame
 * over CAN bus. This allows for sequential and safe module
 * initialization and consent.
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
ULONG distribution_stack[DIST_STACK_ULONG];


/* ------ FC packet handler ------ */

/// Deposits messages into the recovery queue.
SedsResult on_fc_packet(const SedsPacketView *pkt, void *user)
{
  (void) user;
  UINT st = TX_SUCCESS;

  if (!pkt || pkt->ty != SEDS_EP_FLIGHT_CONTROLLER ||
      !pkt->payload || !pkt->payload_len || pkt->payload_len & 3u)
  {
    return SEDS_HANDLER_ERROR;
  }

  for (fu8 k = 0; k < pkt->payload_len; k += sizeof(uint32_t))
  {
    uint32_t msg = U32(pkt->payload[k],   pkt->payload[k+1],
                       pkt->payload[k+2], pkt->payload[k+3]);

    st += tx_queue_send(&shared, &msg, TX_NO_WAIT);
  }

  return st == TX_SUCCESS ? SEDS_OK : SEDS_ERR;
}


/* ------ Local helpers ------ */

/// Locally validate data but do not send reports
/// to the queue. Needed for pre-launch evaluation
/// before crew decides to issue the START command.
static inline fu8
pilot_validate(const struct measurement *data)
{
  enum command st = RAW_DATA;
  
  if (data->baro.alt > MAX_ALT || data->baro.alt < MIN_ALT)
    st += RAW_BAD_ALT;

  if (data->accl.x > MAX_VAX || data->accl.x < MIN_VAX)
    st += RAW_BAD_ACC_X;

  if (data->accl.y > MAX_VAX || data->accl.y < MIN_VAX)
    st += RAW_BAD_ACC_Y;
  
  if (data->accl.z > MAX_VAX || data->accl.z < MIN_VAX)
    st += RAW_BAD_ACC_Z;

  if (data->gyro.x > MAX_ANG || data->gyro.x < MIN_ANG)
    st += RAW_BAD_ANG_X;

  if (data->gyro.y > MAX_ANG || data->gyro.y < MIN_ANG)
    st += RAW_BAD_ANG_Y;

  if (data->gyro.z > MAX_ANG || data->gyro.z < MIN_ANG)
    st += RAW_BAD_ANG_Z;

  return st;
}


/* ------ Distribution Task ------ */

/// Distribution loop that executes before the Flight
/// Computer receives command to launch. After exiting
/// the loop, this function performs final sanity checks
/// and sends ignition signal to the Valve board.
static inline void
pre_launch(struct measurement *payload)
{
  fu8 st = 0;
  enum command cmd = FC_MSG(SYNC);

  task_loop (load(&config, Acq) & ENTER_DIST_CYCLE)
  {
    st = tx_queue_send(&shared, &cmd, TX_NO_WAIT);

    if (st != TX_SUCCESS) {
      log_err("FC:DIST: internal heartbeat failed (%u)", st);
    }

    if (!dma_try_fetch(payload))
    {
      tx_thread_sleep(DIST_SLEEP / 2);
      continue;
    }

    compensate(payload);
    st = pilot_validate(payload);

    if (st != RAW_DATA) {
      log_err("FC:DIST: (PILOT) malformed data (%u)", st);
    }

    log_measurement(SEDS_DT_BAROMETER_DATA, &payload->baro);
    log_measurement(SEDS_DT_GYRO_DATA,      &payload->gyro);
    log_measurement(SEDS_DT_ACCEL_DATA,     &payload->accl);
  }

  /* Request Valve board to ignite the engine.
   * Repeat until the signal is delivered. */
  task_loop (request_ignition() == SEDS_OK);

  log_msg("FC:DIST: Ignition requested, entering flight mode", 50);
}


/// An overview of Flight Computer data distribution.
void distribution_entry(ULONG input)
{
  (void) input;

  struct measurement payload = {0};

  /* Enter pre-launch loop */
  pre_launch(&payload);

  /* Normal post-launch operation begins */
  task_loop (DO_NOT_EXIT)
  {
    if (!dma_try_fetch(&payload))
    {
      tx_thread_sleep(DIST_SLEEP);
      continue;
    }

    compensate(&payload);
    evaluation_put(&payload);

    log_measurement(SEDS_DT_BAROMETER_DATA, &payload.baro);
    log_measurement(SEDS_DT_GYRO_DATA,      &payload.gyro);
    log_measurement(SEDS_DT_ACCEL_DATA,     &payload.accl);
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
                             DIST_INPUT,
                             distribution_stack,
                             DIST_STACK_BYTES,
                             DIST_PRIORITY,
                             /* No preemption */
                             DIST_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("FC:DIST: failed to create task (%u)", st);
  }
}