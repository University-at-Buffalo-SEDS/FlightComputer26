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
enum remote_cmd_compat {

#endif // GNU C + C23

  /* Matches 'Actionable_Decrees' */
  Compat_Deploy_Parachute,
  Compat_Expand_Parachute,
  Compat_Reinit_Sensors,
  Compat_Launch_Signal,
  Compat_Evaluation_Relax,
  Compat_Evaluation_Focus,
  Compat_Evaluation_Abort,
  
  /* Excludes internal config options */
  Compat_Monitor_Altitude,
  Compat_Descent_KF_Feasible,
  Compat_Consecutive_Samples,
  Compat_Confirm_Altitude,
  Compat_Reset_Failures,
  Compat_Validate_Measms,
  Compat_Using_Ascent_KF,

  Compat_Renormalize_Quat_1,
  Compat_Renormalize_Quat_2,
  Compat_Renormalize_Quat_4,
  Compat_Renormalize_Quat_8,

  Compat_Abort_After_15,
  Compat_Abort_After_40,
  Compat_Abort_After_70,

  Compat_Reinit_After_12,
  Compat_Reinit_After_26,
  Compat_Reinit_After_44,

  Compat_Messages
};

#if !defined(__GNUC__) && __STDC_VERSION__ < 202311L
  _Static_assert(sizeof(enum remote_cmd_compat) == sizeof(uint8_t), "");
#endif


/// O(1) map between byte code and FC message.
static const enum message extmap[Compat_Messages] = {
    Deploy_Parachute,    Expand_Parachute,   Reinit_Sensors,
    Launch_Signal,       Evaluation_Relax,   Evaluation_Focus,
    Evaluation_Abort,    Monitor_Altitude,   Descent_KF_Feasible,
    Consecutive_Samples, Confirm_Altitude,   Reset_Failures,
    Validate_Measms,     Using_Ascent_KF,    Renormalize_Quat_1,
    Renormalize_Quat_2,  Renormalize_Quat_4, Renormalize_Quat_8,
    Abort_After_15,      Abort_After_40,     Abort_After_70,
    Reinit_After_12,     Reinit_After_26,    Reinit_After_44
};


#define MIN_CMD_SIZE 1

static inline enum message decode_cmd(const uint8_t *raw)
{
  return extmap[*raw];
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

  if (load(&config, Rlx) & static_option(Using_Ascent_KF)) {
    /* GPS data is not needed. */
    return SEDS_OK;
  }

  if (len != 3 * sizeof(float)) {
    enum message cmd = FC_MSG(GPS_Malformed);
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    return SEDS_ERR;
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

/// Fetches latest available GPS packet, or checks for timeout.
static inline fu8 fetch_gps_data()
{
  tx_mutex_get(&mu_gps, TX_WAIT_FOREVER);

  if (gps_cons_serial == gps_prod_serial) {
    tx_mutex_put(&mu_gps);

    if (timer_exchange(IntervalGPS) > GPS_DELAY_MS) {
      enum message cmd = FC_MSG(GPS_Delayed);
      tx_queue_send(&shared, &cmd, TX_NO_WAIT);
    }

    return 0;
  }
  
  payload.d.gps = gps_bucket;
  gps_cons_serial = gps_prod_serial;

  tx_mutex_put(&mu_gps);
  timer_update(IntervalGPS);

  return 1;
}


/// Checks for timeout but does not fetch GPS data.
/// Provides early inference of GPS availability.
static inline void check_for_gps_packet()
{
  if (timer_exchange(IntervalGPS) > GPS_DELAY_MS) {
    enum message cmd = FC_MSG(GPS_Delayed);
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

#endif // GPS_AVAILABLE


/// Deposits one or multiple messages into the recovery queue.
static inline SedsResult
handle_gnd_command(const uint8_t *data, size_t len)
{
  UINT st = TX_SUCCESS;
  enum message msg;

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

/// Locally validate all data but do not send reports to the queue.
static inline fu8 test_validate_all()
{
  enum message st = Sensor_Measm_Code;

  if (payload.baro.alt > MAX_ALT || payload.baro.alt < MIN_ALT)
    st += Bad_Altitude;

  if (payload.d.accl.x > MAX_ACC || payload.d.accl.x < MIN_ACC)
    st += Bad_Accel_X;

  if (payload.d.accl.y > MAX_ACC || payload.d.accl.y < MIN_ACC)
    st += Bad_Accel_Y;

  if (payload.d.accl.z > MAX_ACC || payload.d.accl.z < MIN_ACC)
    st += Bad_Accel_Z;

  if (payload.gyro.x > MAX_DPS || payload.gyro.x < MIN_DPS)
    st += Bad_Attitude_X;

  if (payload.gyro.y > MAX_DPS || payload.gyro.y < MIN_DPS)
    st += Bad_Attitude_Y;

  if (payload.gyro.z > MAX_DPS || payload.gyro.z < MIN_DPS)
    st += Bad_Attitude_Z;

#if GPS_AVAILABLE
  if (payload.d.gps.x > MAX_GPS_X || payload.d.gps.x < MIN_GPS_X)
    st += Bad_Lattitude;

  if (payload.d.gps.y > MAX_GPS_Y || payload.d.gps.y < MIN_GPS_Y)
    st += Bad_Longtitude;

  if (payload.d.gps.z > MAX_GPS_Z || payload.d.gps.z < MIN_GPS_Z)
    st += Bad_Sea_Level;

#endif // GPS_AVAILABLE

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
  enum message cmd = FC_MSG(Sensor_Measm_Code);

  task_loop (load(&config, Acq) & static_option(Launch_Triggered))
  {
    st = tx_queue_send(&shared, &cmd, TX_NO_WAIT);

    if (st != TX_SUCCESS) {
      log_err("FC:DIST: (PILOT) heartbeat failed (%u)", st);
    }

    if (!dma_try_fetch(&payload, 0))
    {
      tx_thread_sleep(DIST_SLEEP_NO_DATA);
      continue;
    }

    compensate(&payload, 0);
    st = test_validate_all();

    if (st != Sensor_Measm_Code) {
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
  if (!(load(&config, Acq) & static_option(Launch_Triggered))) {
    pre_launch();
  }

  start: task_loop (DO_NOT_EXIT)
  {
    fu8 for_ukf = load(&config, Acq) & static_option(Using_Ascent_KF);

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
#ifdef GPS_AVAILABLE
      check_for_gps_packet();
#endif
    }

#ifdef GPS_AVAILABLE
    else
    {
      while (!fetch_gps_data())
      {
        if (load(&config, Acq) & static_option(Using_Ascent_KF))
        {
          goto start;
        }
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

  st = tx_mutex_create(&mu_gps, "GPS bucket mutex", TX_NO_INHERIT);

  if (st != TX_SUCCESS) {
    log_die("FC:DIST: failed to create GPS bucket mutex (%u)", st);
  }
}