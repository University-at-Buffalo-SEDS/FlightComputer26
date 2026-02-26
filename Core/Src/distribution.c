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
 * being (in turn) started by the Recovery Task, which resumes
 * after receiving the 'START' signal on its blocking queue,
 * which (in turn) is deposited there by the telemetry handler
 * (below). This allows for sequential and safe module
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

#include "kalman.h"
#include "platform.h"
#include "evaluation.h"
#include "recovery.h"
#include "dma.h"

TX_THREAD distribution_task;
ULONG distribution_stack[DIST_STACK_ULONG];


/* ------ Global and static storage ------ */

#define id    "DI "
#define pilot "PILOT "

/* Latest logged measurement */
struct measurement payload = {0};

struct coords rail = {0};

/* ------ Global and static storage ------ */


/* ------ Packet handling definitions ------ */

#ifdef TELEMETRY_ENABLED

#define telid "TE "

#ifdef TELEMETRY_CMD_COMPAT

enum remote_cmd_compat : uint8_t {

  /* Matches 'Actionable_Decrees' */
  Compat_Deploy_Parachute,
  Compat_Expand_Parachute,
  Compat_Reinit_Sensors,
  Compat_Launch_Signal,
  Compat_Evaluation_Relax,
  Compat_Evaluation_Focus,
  Compat_Evaluation_Abort,
  Compat_Reinit_Barometer,
  Compat_Enable_IMU,
  Compat_Disable_IMU,
  
  /* Excludes internal config options */
  Compat_Monitor_Altitude,
  Revoke_Monitor_Altitude,
  Compat_Consecutive_Samples,
  Revoke_Consecutive_Samples,
  Compat_Reset_Failures,
  Revoke_Reset_Failures,
  Compat_Validate_Measms,
  Revoke_Validate_Measms,

  Compat_Abort_After_15,
  Compat_Abort_After_40,
  Compat_Abort_After_70,

  Compat_Reinit_After_12,
  Compat_Reinit_After_26,
  Compat_Reinit_After_44,

  Compat_Messages
};

/* O(1) map between byte code and FC message */
static const enum message extmap[Compat_Messages] = {
        Deploy_Parachute,
        Expand_Parachute,
        Reinit_Sensors,
        Launch_Signal,
        Evaluation_Relax,
        Evaluation_Focus,
        Evaluation_Abort,
        Reinit_Barometer,
        Enable_IMU,
        Disable_IMU,

        Monitor_Altitude,
        revoke(Monitor_Altitude),
        Consecutive_Samples,
        revoke(Consecutive_Samples),
        Reset_Failures,
        revoke(Reset_Failures),
        Validate_Measms,
        revoke(Validate_Measms),

        Abort_After_15,
        Abort_After_40,
        Abort_After_70,
        Reinit_After_12,
        Reinit_After_26,
        Reinit_After_44,
};

#define MIN_CMD_SIZE 1

static inline enum message decode_cmd(const uint8_t *raw)
{
  return *raw < Sensors ? extmap[*raw] : Invalid_Message;
}

#else // TELEMETRY_CMD_COMPAT

#define MIN_CMD_SIZE 4

static inline enum message decode_cmd(const uint8_t *raw)
{
  return (enum message) U32(raw[0], raw[1], raw[2], raw[3]);
}

#endif // TELEMETRY_CMD_COMPAT

/* ------ Packet handling definitions ------ */


/* ------ GPS handling functions ------ */

#ifdef GPS_AVAILABLE

#define GPS_RING_SIZE 4
#define GPS_RING_MASK (GPS_RING_SIZE - 1)

/* 0-7:  amount of new entries,
 * 8-15: 'consumer lock' index. 0xFF when unlocked. */
static atomic_uint_fast16_t gps_mask = 0xFF00u;
static struct coords gps_ring[GPS_RING_SIZE] = {0};

/*
 * Deterministically writes GPS data struct into
 * a free bucket. Guards against locks, dropped
 * packets, and inadequate consumer. Safe when
 * the calling thread is asynchronously cancelled.
 */
static inline void
enqueue_gps_data(const uint8_t *buf)
{
  static fu8 idx = 0;

  fu8 i = idx;
  memcpy(&gps_ring[i], buf, sizeof(struct coords));

  i = (i + 1) & RING_MASK;
  fu8 cons = load(&gps_mask, Acq) >> 8;

  if (i != cons) {
    fetch_add(&gps_mask, 1, Rel);
    idx = i;
  }
}

/*
 * Fetches latest available GPS packet, or checks
 * for timeout. Returns the amount of new entries
 * added since last call. Safe when the calling
 * thread is asynchronously cancelled.
 */
static inline fu8
fetch_gps_data(struct coords *buf)
{
  static fu8 idx = UINT_FAST8_MAX;

  fu16 i = idx;
  fu8 n = (fu8) swap(&gps_mask, i << 8, AcqRel);
  
  if (!n) {
    fetch_or(&gps_mask, CLEAR_IDX, Rlx);
    return 0;
  }

  i = (i + n) & RING_MASK;
  idx = i;

  *buf = gps_ring[i];

  fetch_or(&gps_mask, CLEAR_IDX, Rlx);
  return n;
}

/*
 * GPS sanity check against NEO-M9N data range.
 */
static inline enum message
validate_gps_absolute(const struct coords *gps)
{
  enum message st = 0;

  if (gps->x > MAX_LAT || gps->x < MIN_LAT) {
    st |= Bad_Lattitude;
  }
  if (gps->y > MAX_LON || gps->y < MIN_LON) {
    st |= Bad_Longtitude;
  }
  if (gps->z > MAX_SEA || gps->z < MIN_SEA) {
    st |= Bad_Sea_Level;
  }

  return st;
}

/*
 * GPS sanity check against launch coordinates.
 * Tolerance of 1.41 degree makes it a sanity check.
 * (We are basically checking for being within ~150
 * km from Midland, TX).
 */
static inline enum message
validate_gps_relative(const struct coords *gps)
{
  if (!lat_within_launch_site(gps->x) ||
      !lon_within_launch_site(gps->y))
  {
    log_err(telid "GPS coords beyond launch site:"
            " LAT: %f, LON: %f", gps->x, gps->y); 
  }

  if (gps->z > MAX_SEA || gps->z < MIN_SEA) {
    return Bad_Sea_Level;
  }

  return 0;
}

/*
 * Checks received size and invokes appropriate validator.
 */
static inline fu32
validate_gps_serial(const uint8_t *data, size_t len)
{
  enum message rep = fc_mask(GPS_Data_Code);

  if (len != sizeof(struct coords))
  {
    rep |= GPS_Malformed;
  }
  else
  {
    fu32 conf = load(&config, Acq);

    if (conf & option(Validate_Measms))
    {
      rep |= conf & option(Launch_Triggered)
          ? validate_gps_absolute((const struct coords *)data)
          : validate_gps_relative((const struct coords *)data);
    }
  }

  return rep;
}

/*
 * Accumulates helpers refresh RF board heartbeat,
 * validate GPS data, and enqueue valid packet.
 */
static inline SedsResult
handle_gps_data(const uint8_t *data, size_t len)
{
  timer_update(HeartbeatRF);
  
  fu32 rep = validate_gps_serial(data, len);

  if (rep != fc_mask(GPS_Data_Code))
  {
    tx_queue_send(&shared, &rep, TX_NO_WAIT);
    return SEDS_ERR;
  }

  enqueue_gps_data(data);

  return SEDS_OK;
}

/*
 * Converts GPS coordinates into distance from
 * launch rail. This is how Descent KF expects GPS.
 */
static inline void
distance_from_rail(struct coords *gps)
{
  // TODO Dynamics
}

#endif // GPS_AVAILABLE

/* ------ GPS handling functions ------ */


/* ------ Packet handling functions ------ */

#define INVALID_MESSAGE_STATUS 0xFFu

/*
 * Deposits one or multiple messages into the recovery queue.
 */
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
    st += (msg != Invalid_Message) ? tx_queue_send(&shared, &msg, TX_NO_WAIT)
                                   : INVALID_MESSAGE_STATUS;
  }

  tx_thread_priority_change(&telemetry_thread, TLMT_PRIORITY, &tlmt_old_pr);

#else
  msg = decode_cmd(data);
  st = (msg != Invalid_Message) ? tx_queue_send(&shared, &msg, TX_NO_WAIT)
                                : INVALID_MESSAGE_STATUS;

#endif // MESSAGE_BATCHING_ENABLED

  return st == TX_SUCCESS ? SEDS_OK : SEDS_ERR;
}

/*
 * Calls appropriate handler based on sender id.
 * Invoked from the Telemetry thread.
 */
SedsResult on_fc_packet(const SedsPacketView *pkt, void *user)
{
  (void) user;

  if (!pkt || pkt->ty != SEDS_EP_FLIGHT_CONTROLLER ||
      !pkt->payload || !pkt->payload_len ||
      !pkt->sender || !pkt->sender_len)
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
    return handle_gps_data(pkt->payload, pkt->payload_len);
  }
#endif

  else
  {
    return SEDS_HANDLER_ERROR;
  }
}

#endif // TELEMETRY_ENABLED

/* ------ Packet handling functions ------ */


/* ------ Sensor data sanity checks ------ */

/*
 * IMU sanity check against BMI088 data range.
 */
static inline enum message
validate_imu(const struct coords *gyro,
             const struct coords *accl, fu32 conf)
{
  enum message st = fc_mask(Sensor_Measm_Code);

  if (conf & option(Validate_Measms))
  {
    if (accl->x > MAX_ACC || accl->x < MIN_ACC) {
      st |= Bad_Accel_X;
    }
    if (accl->y > MAX_ACC || accl->y < MIN_ACC) {
      st |= Bad_Accel_Y;
    }
    if (accl->z > MAX_ACC || accl->z < MIN_ACC) {
      st |= Bad_Accel_Z;
    }
    if (gyro->x > MAX_DPS || gyro->x < MIN_DPS) {
      st |= Bad_Attitude_X;
    }
    if (gyro->y > MAX_DPS || gyro->y < MIN_DPS) {
      st |= Bad_Attitude_Y;
    }
    if (gyro->z > MAX_DPS || gyro->z < MIN_DPS) {
      st |= Bad_Attitude_Z;
    }
  }

  return st;
}

/*
 * Barometer sanity check against BMP390 data range.
 */
static inline enum message
validate_baro(const struct baro *baro, fu32 conf)
{
  enum message st = fc_mask(Sensor_Measm_Code);

  if (conf & option(Validate_Measms))
  {
    if (baro->p > MAX_PRS || baro->p < MAX_PRS) {
      st |= Bad_Pressure;
    }
    if (baro->alt > MAX_ALT || baro->alt < MIN_ALT) {
      st |= Bad_Altitude;
    }
  }

  return st;
}

/*
 * Call and aggregate statuses from all validator functions.
 */
static inline enum message                                  IREC26_unused
validate_all(const struct measurement *buf, fu32 conf)
{
  return validate_baro(&buf->baro, conf) |
         validate_imu(&buf->gyro, &buf->d.accl, conf);
}

/* ------ Sensor data sanity checks ------ */


/* ------ Distribution Task ------ */

/*
 * Distribution loop that executes before the Flight
 * Computer receives command to launch. After exiting
 * the loop, this function performs final sanity checks
 * and sends ignition signal to the Valve board.
 */
static inline void pre_launch(void)
{
  fu32 st = 0, counter = 0;

  float accum_baro = 0, accum_gps = 0;
  fu32 ctr_baro = 0, ctr_gps = 0;

  fu32 conf = load(&config, Acq);

  task_loop (conf & option(Launch_Triggered))
  {
    if (st != TX_SUCCESS) {
      log_err(pilot "heartbeat failed: %u", st);
    }

    if (!dma_fetch_imu(&payload.gyro, &payload.d.accl))
    {
      tx_thread_relinquish();
      continue;
    }

    st = validate_imu(&payload.gyro, &payload.d.accl, conf);

    if (st == Sensor_Measm_Code) {
      compensate_accl(&payload.d.accl);
      log_measurement(SEDS_DT_GYRO_DATA,  &payload.gyro);
      log_measurement(SEDS_DT_ACCEL_DATA, &payload.d.accl);
    }
    else {
      log_err(pilot "malformed IMU data: %u", st);
    }

    if (dma_fetch_baro(&payload.baro))
    {
      accum_baro += fsec(timer_exchange(IntervalBaro));
      ++ctr_gps;

      st = validate_baro(&payload.baro, conf);

      if (st == Sensor_Measm_Code) {
        compensate_baro(&payload.baro);
        log_measurement(SEDS_DT_BAROMETER_DATA, &payload.baro);
      }
      else {
        log_err(pilot "malformed Baro data: %u", st);
      }
    }

#ifdef GPS_AVAILABLE
    if (conf & option(GPS_Available) &&
        fetch_gps_data(&payload.d.gps))
    {
      accum_gps += fsec(timer_exchange(IntervalGPS));
      ++ctr_gps;
      /* GPS data is validated by the Telemetry thread,
       * and reported by the RF board. */

      /* As long as we are stationary, update launch coordinates 
       * with newest available. */
      memcpy(&rail, &payload.d.gps, sizeof(struct coords));

      if (counter != 0 &&
          (fabsf(rail.x - payload.d.gps.x) > GPS_RAIL_TOLER ||
           fabsf(rail.y - payload.d.gps.y) > GPS_RAIL_TOLER))
      {
        log_err(id "contraversial launch coords: "
                   "LAT: %f, LON: %f", rail.x, rail.y);
      }
    }
#endif // GPS_AVAILABLE

    if (!(++counter & 31)) {
      // FIXME ask to provide explicit packet type for
      // rates.
      if (ctr_baro > 0) {
        log_err(pilot "gathering baro every %f sec", accum_baro / ctr_baro);
      }
      if (ctr_gps > 0) {
        log_err(pilot "gathering gps every %f sec", accum_gps / ctr_gps);
      }
    }

    conf = load(&config, Acq);
  }

  /* Request Valve board to ignite the engine. */
  task_loop (request_ignition() == SEDS_OK);

  log_msg(id "ignition requested, in flight mode", mlen(34));
}

/*
 * Data cycle for the Ascent filter.
 */
static inline void ascent_cycle(fu32 conf)
{
  fu32 st;

  if (!dma_fetch_imu(&payload.gyro, &payload.d.accl))
  {
    /* Wait for IMU measurements */
    tx_thread_relinquish();
    return;
  }

  st = validate_imu(&payload.gyro, &payload.d.accl, conf);

  if (st != fc_mask(Sensor_Measm_Code)) {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
    return;
  }

  compensate_accl(&payload.d.accl);
  sh.dt = fsec(timer_exchange(AscentKF));

  ascent_predict(sh.dt);

  if (!dma_fetch_baro(&payload.baro)) {
    /* Run Predict in the meanwhile */
    return;
  }

  st = validate_baro(&payload.baro, conf);

  if (st != fc_mask(Sensor_Measm_Code)) {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
    return;
  }

  if (conf & option(Eval_Focus_Flag)) {
    tx_semaphore_put(&eval_focus_mode);
  }
  else {
    ascent_update(sh.dt);
    evaluate_rocket_state(conf);
  }
}

/*
 * Data cycle for the Descent filter.
 */
static inline void descent_cycle(fu32 conf)
{
  fu32 st;

  if (!dma_fetch_baro(&payload.baro)) {
    tx_thread_relinquish();
    return;
  }

  st = validate_baro(&payload.baro, conf);
  if (st != fc_mask(Sensor_Measm_Code)) {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
    return;
  }

#ifdef GPS_AVAILABLE
  if (conf & option(GPS_Available) &&
      !fetch_gps_data(&payload.d.gps))
  {
    tx_thread_relinquish();
    return;
  }
#endif // GPS_AVAILABLE

  sh.dt = fsec(timer_exchange(DescentKF));

  // call descent filter
}

/*
 * Distribution task entry that also serves as
 * an overview of the Flight Computer data distribution.
 * This function is idempotent and can be reentered whenever
 * any critical conditions arise as deemed by Recovery task.
 */
void distribution_entry(ULONG input)
{
  (void) input;

  log_msg(id "started", mlen(7));

  fu32 conf = load(&config, Acq);

  /* Enter pre-launch loop only once. */
  if (!(conf & option(Launch_Triggered))) {
    pre_launch();
  }

  task_loop (DO_NOT_EXIT)
  {
    conf = load(&config, Acq);

    conf & option(Using_Ascent_KF) ? ascent_cycle(conf)
                                   : descent_cycle(conf);
  }
}

/*
 * Creates a preemptive, cooperative Distribution
 * task with defined parameters.
 */
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
    log_die(id "task creation failure: %u", st);
  }
}