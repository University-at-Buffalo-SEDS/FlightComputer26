/*
 * Distribution Task
 *
 * Serves as the Flight Computer coordination module.
 * Provides telemetry handler for GND commands and GPS data. 
 *
 * This task includes a packet handler that is comptime
 * subscribed to messages incoming to the FLIGHT_CONTROLLER
 * endpoint. This handler deposits messages into the Recovery
 * queue and receives GPS data from the telemetry router.
 *
 * Before entering the normal operation (flight) loop,
 * this task runs a pilot (demo) loop that fetches and
 * validates data but does not pass it for evaluation.
 * This is needed to verify that sensors are functional
 * and to inform the Ground Station of any issues.
 *
 * The pilot loop is left and ignition is requested from the
 * Valve board upon receiving the 'ENTER_DIST_CYCLE' atomic
 * flag from the Evaluation Task, which sets this flag after
 * being (in turn) woken on a semaphore by the Recovery Task,
 * which sends it (in turn) after receiving the 'START' signal
 * on its queue, which (in turn) is deposited there by the
 * telemetry handler (below). This allows for sequential and
 * safe module initialization and consent.
 *
 * When fetching a pack of data (barometer, gyroscope, and
 * accelerometer readings) from DMA buffers, it is not
 * guaranteed that all three readings will be ready at once.
 * The underlying DMA fetch function is thus called repetedly
 * from the main loop until it (the function) notifies the
 * caller (this task) that the buffer is ready and it can
 * proceed. This is made to simplify synchronization within
 * the Interrupt Service Routine, where ThreadX API, and
 * therefore its wait/wake service, is unavailable. When
 * using Descent KF, this task will inform DMA that it can
 * skip Accel and Gyro readings, but will yield indefinitely
 * if no GPS data is available, until it either becomes
 * available or Recovery task falls back to using Ascent KF.
 *
 * The data is then passed for preliminary compensation 
 * (provided by device drivers), and placed inside the
 * data evaluation buffer and telemetry (UART and SD) queues.
 */

#include "platform.h"
#include "evaluation.h"
#include "recovery.h"
#include "dma.h"

TX_THREAD distribution_task;
ULONG distribution_stack[DIST_STACK_ULONG];


/* ------ Static ------ */

/// Latest logged measurement.
static struct measurement payload = {0};


#ifdef TELEMETRY_ENABLED

/* ------ FC packet handling ------ */


#ifdef TELEMETRY_CMD_COMPAT

#if defined(__GNUC__) || __STDC_VERSION__ >= 202311L
enum remote_cmd_compat : uint8_t {

#else
enum command {

#endif // GNU C + C23

  Launch,
  Fire_Parachute,
  Expand_Parachute,
  Reinitialize_Sensors,

  /* Preempt Evaluation as per its time slice. */
  Evaluation_Relax,

  /* Do not preempt Evaluation
   * during resource-heavy KF iteration. */
  Evaluation_Focus,

  /* Log but do not evaluate data. Ends Evaluation task. */
  Evaluation_Abort,

  /* Regardless of state evaluated, monitor
   * altitude changes. */
  Always_Check_Altitude,

  /* How often to renormalize quaternion column.
   * The number is directly proportional to the
   * speed of the Predict step of Ascent KF.
   *
   * Why having this? Ascent KF is much richer 
   * and slower than Descent KF. */
  Renormalize_Every_1,
  Renormalize_Every_2,
  Renormalize_Every_4,
  Renormalize_Every_8,

  /* Whether to NOT reset fail count on a clean data report. 
   * DANGER ZONE: only enable this if Flight Computer is
   * under complete and reliable control from the ground. */
  Accumulate_Failures,

  /* How many failures should trigger reinit / Eval abort */
  Fails_To_Reinit_5,
  Fails_To_Reinit_12,
  Fails_To_Reinit_20,
  Fails_To_Abort_10,
  Fails_To_Abort_20,
  Fails_To_Abort_50,

  /* Manually set FC to always use unscented filter. */
  Use_Ascent_KF,

  Compat_Commands,
};

#if !defined(__GNUC__) && __STDC_VERSION__ < 202311L
  _Static_assert(sizeof(enum remote_cmd_compat) == sizeof(uint8_t), "");
#endif


static const enum command cmdmap[Compat_Commands] = {
  START, FIRE_PYRO, FIRE_REEF, RECOVER, EVAL_RELAX, EVAL_FOCUS,
  EVAL_ABORT, ALT_CHECKS, CMD_RENORM_QUATERN_1, CMD_RENORM_QUATERN_2,
  CMD_RENORM_QUATERN_4, CMD_RENORM_QUATERN_8, ACCUM_FAILS, REINIT_5,
  REINIT_12, REINIT_20, ABORT_10, ABORT_20, ABORT_50, USE_ASCENT,
};


#define MIN_CMD_SIZE 1

static inline enum command decode_cmd(const uint8_t *raw)
{
  return cmdmap[*raw];
}


#else

#define MIN_CMD_SIZE 4

static inline enum command decode_cmd(const uint8_t *raw)
{
  return (enum command) U32(raw[0], raw[1], raw[2], raw[3]);
}


#endif // TELEMETRY_CMD_COMPAT


#ifdef GPS_AVAILABLE

static struct coords gps_bucket = {0};
static fu8 gps_prod_serial = 0;
static fu8 gps_cons_serial = 0;

static TX_MUTEX mu_gps;


/// Called from within the Telemetry thread.
static inline SedsResult
handle_gps_data(const uint8_t *data, size_t len, uint64_t ts)
{
  static fu64 gps_ref_time = 0;

  timer_update(HeartbeatRF);

  if (load(&unscented, Rlx)) {
    /* GPS data is not needed. */
    return SEDS_OK;
  }

  if (gps_ref_time == 0)
  {
    gps_ref_time = ts*1000UL - hal_time_ms();
  }
  else if (ts*1000UL - gps_ref_time > GPS_TIME_DRIFT_MS)
  {
    log_err("FC:TLMT: warning: using outdated GPS data.");
  }

  tx_mutex_get(&mu_gps, TX_WAIT_FOREVER);

  memcpy(&gps_bucket, data, 3 * sizeof(float));
  ++gps_prod_serial;

  tx_mutex_put(&mu_gps);

  return SEDS_OK;
}

/// Replaces accel data with GPS data, if available. Otherwise hangs.
/// Signals to Recovery task if plausible internal between GPS packets
/// was exceeded. Called from within the Distribution task.
static inline fu8 fetch_gps_data()
{
  tx_mutex_get(&mu_gps, TX_WAIT_FOREVER);

  if (gps_cons_serial == gps_prod_serial) {
      tx_mutex_put(&mu_gps);

    if (timer_fetch_update(IntervalGPS) > GPS_DELAY_MS) {
      enum command cmd = FC_MSG(GPS_DELAY);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }

    return 0;
  }
  
  payload.d.gps = gps_bucket;
  gps_cons_serial = gps_prod_serial;

  tx_mutex_put(&mu_gps);

  return 1;
}

#endif // GPS_AVAILABLE

/// Deposits one or multiple messages into the recovery queue.
static inline SedsResult
handle_gnd_command(const uint8_t *data, size_t len)
{
  UINT st = TX_SUCCESS;
  enum command msg;

#ifdef MESSAGE_BATCHING_ENABLED
  UINT tlmt_old_pr;

  /* This avoids context switch to Recovery when enqueueing a batch. */
  tx_thread_priority_change(&telemetry_thread, 0, &tlmt_old_pr);

  for (fu16 k = 0; k < len; k += MIN_CMD_SIZE)
  {
    msg = decode_cmd(data + k);
    st += tx_queue_send(&shared, &msg, TX_NO_WAIT);
  }

  tx_thread_priority_change(&telemetry_thread, TLMT_PRIORITY, &tlmt_old_pr);

#else
  msg = decode_cmd(data);
  st = tx_queue_send(&shared, &msg, TX_NO_WAIT);

#endif // MESSAGE_BATCHING_ENABLED

  return st == TX_SUCCESS ? SEDS_OK : SEDS_ERR;
}


/// Calls appropriate handler based on sender id.
SedsResult on_fc_packet(const SedsPacketView *pkt, void *user)
{
  (void) user;

  if (!pkt || pkt->ty != SEDS_EP_FLIGHT_CONTROLLER || !pkt->payload
      || !pkt->payload_len || !pkt->sender || !pkt->sender_len)
  {
    return SEDS_HANDLER_ERROR;
  }

  /* Heuristic: 'G' -> 'GS', 'R' -> 'RF' */
  if (pkt->sender[0] == 'G')
  {
    return handle_gnd_command(pkt->payload, pkt->payload_len);
  }

#ifdef GPS_AVAILABLE
  else if (pkt->sender[0] == 'R')
  {
    return handle_gps_data(pkt->payload, pkt->payload_len, pkt->timestamp);
  }
#endif

  else
  {
    return SEDS_HANDLER_ERROR;
  }
}


#endif // TELEMETRY_ENABLED


/* ------ Local helpers ------ */

/// Locally validate data but do not send reports
/// to the queue. Needed for pre-launch evaluation
/// before crew decides to issue the START command.
static inline fu8 pilot_validate()
{
  enum command st = RAW_DATA;
  
  if (payload.baro.alt > MAX_ALT || payload.baro.alt < MIN_ALT)
    st += RAW_BAD_ALT;

  if (payload.d.accl.x > MAX_VAX || payload.d.accl.x < MIN_VAX)
    st += RAW_BAD_ACC_X;

  if (payload.d.accl.y > MAX_VAX || payload.d.accl.y < MIN_VAX)
    st += RAW_BAD_ACC_Y;
  
  if (payload.d.accl.z > MAX_VAX || payload.d.accl.z < MIN_VAX)
    st += RAW_BAD_ACC_Z;

  if (payload.gyro.x > MAX_ANG || payload.gyro.x < MIN_ANG)
    st += RAW_BAD_ANG_X;

  if (payload.gyro.y > MAX_ANG || payload.gyro.y < MIN_ANG)
    st += RAW_BAD_ANG_Y;

  if (payload.gyro.z > MAX_ANG || payload.gyro.z < MIN_ANG)
    st += RAW_BAD_ANG_Z;

  return st;
}


/* ------ Distribution Task ------ */

/// Distribution loop that executes before the Flight
/// Computer receives command to launch. After exiting
/// the loop, this function performs final sanity checks
/// and sends ignition signal to the Valve board.
static inline void pre_launch()
{
  fu8 st = 0;
  enum command cmd = FC_MSG(SYNC);

  task_loop (load(&config, Acq) & ENTER_DIST_CYCLE)
  {
    st = tx_queue_send(&shared, &cmd, TX_NO_WAIT);

    if (st != TX_SUCCESS) {
      log_err("FC:DIST: internal heartbeat failed (%u)", st);
    }

    if (!dma_try_fetch(&payload, 0))
    {
      tx_thread_sleep(DIST_SLEEP_NO_DATA);
      continue;
    }

    compensate(&payload, 0);
    st = pilot_validate();

    if (st != RAW_DATA) {
      log_err("FC:DIST: (PILOT) malformed data (%u)", st);
    }

    log_measurement(SEDS_DT_BAROMETER_DATA, &payload.baro);
    log_measurement(SEDS_DT_GYRO_DATA,      &payload.gyro);
    log_measurement(SEDS_DT_ACCEL_DATA,     &payload.d.accl);
  }

  /* Request Valve board to ignite the engine. */
  task_loop (request_ignition() == SEDS_OK);

  log_msg("FC:DIST: Ignition requested, entering flight mode", 50);
}


/// Distribution task entry that also serves as
/// an overview of the Flight Computer data distribution.
/// This function is idempotent and can be reentered whenever
/// any critical conditions arise as deemed by Recovery task.
void distribution_entry(ULONG input)
{
  (void) input;

  /* Enter pre-launch loop only once. */
  if (!(load(&config, Acq) & ENTER_DIST_CYCLE)) {
    pre_launch();
  }

  task_loop (DO_NOT_EXIT)
  {
    fu8 for_ukf = load(&unscented, Acq);

    /* Do not wait for Gyro & Accel if we use Descent KF. */
    fu8 skip_mask = for_ukf ? 0 : GYRO_DONE | ACCL_DONE;

    if (!dma_try_fetch(&payload, skip_mask))
    {
      tx_thread_sleep(DIST_SLEEP_NO_DATA);
      continue;
    }

    compensate(&payload, skip_mask);

    log_measurement(SEDS_DT_BAROMETER_DATA, &payload.baro);

    if (for_ukf)
    {
      log_measurement(SEDS_DT_GYRO_DATA,  &payload.gyro);
      log_measurement(SEDS_DT_ACCEL_DATA, &payload.d.accl);
    }
#ifdef GPS_AVAILABLE
    else
    {
      while (!fetch_gps_data() && !load(&unscented, Acq))
      {
        /* GPS data is required for Descent filter to proceed. */
        tx_thread_relinquish();
      }
    }
#endif

    evaluation_put(&payload);
  }
}


/// Creates a preemptive, Distribution task with defined parameters.
void create_distribution_task(void)
{
  UINT st = tx_thread_create(&distribution_task,
                             "Distribution Task",
                             distribution_entry,
                             DIST_INPUT,
                             distribution_stack,
                             DIST_STACK_BYTES,
                             DIST_PRIORITY,
                             /* No preemption threshold */
                             DIST_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("FC:DIST: failed to create task (%u)", st);
  }

  st = tx_mutex_create(&mu_gps, "GPS bucket mutex.", TX_NO_INHERIT);

  if (st != TX_SUCCESS) {
    log_die("FC:DIST: failed to create GPS bucket mutex (%u)", st);
  }
}