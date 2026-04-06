/*
 * Distribution Task
 *
 * Serves as the Flight Computer coordination module.
 * Distributes data across Kalman filters and generates
 * sensor data reports for the Recovery task.
 * Provides telemetry handler for GND commands and GPS data.
 *
 * This task includes a packet handler that is subscribed to
 * messages incoming to the FLIGHT_CONTROLLER endpoint.
 * This handler deposits messages into the Recovery
 * queue and receives GPS data from the telemetry router.
 *
 * Before entering the normal operation (flight) loop,
 * this task runs a pilot (demo) loop that fetches and
 * validates data but does not pass it for evaluation.
 * This is needed to verify that sensors are functional
 * and to inform the Ground Station of any issues.
 *
 * The pilot loop is left and ignition is requested from the
 * Valve board upon receiving the 'Launch_Triggered' atomic
 * flag from the Evaluation Task, which sets this flag after
 * being in turn started by the Recovery Task, which resumes
 * after receiving the 'Launch' signal on its blocking queue,
 * which in turn is deposited there by the telemetry handler
 * (below). This allows for sequential and safe module
 * initialization and consent.
 *
 * When fetching a pack of data (barometer, gyroscope, and
 * accelerometer readings) from DMA buffers, it is not
 * guaranteed that all three readings will be ready when
 * required by the distribution task (although DMA task tries
 * its best to start a transfer for the most likely needed sensor).
 * The Distribution task will attempt to proceed with minimum
 * available data. See KF function declarations in 'kalman.h'
 * to see the required minimum for each stage of each filter,
 * as well as logic below.
 *
 * The data is then passed for filtering and evaluation, and
 * then placed inside the telemetry (UART and SD) queues.
 */

#include "kalman.h"
#include "platform.h"
#include "evaluation.h"
#include "recovery.h"
#include "dma.h"
#include "sweetbench.h"

TX_THREAD distribution_task;


/* ------ Global and static storage ------ */

#define id "DI "
#define pilot "PI "
#define pi_bar "BAR"
#define pi_gps "GPS"

/* Latest logged measurement */
struct measurement payload = {0};

#ifdef GPS_AVAILABLE

/* Struct that stores launch coordinates */
static struct coords rail = {0};

#endif // GPS_AVAILABLE

/* ------ Global and static storage ------ */


/* ------ Packet handling definitions ------ */

#ifdef TELEMETRY_ENABLED

#define telid "TE "

#ifdef TELEMETRY_CMD_COMPAT

enum remote_cmd_compat : uint8_t
{
  /* Production */
  Compat_Deploy_Parachute,
  Compat_Expand_Parachute,
  Compat_Reinit_Sensors,
  Compat_Launch_Signal,

  /* HITL */
  Compat_Evaluation_Relax,
  Compat_Evaluation_Focus,
  Compat_Evaluation_Abort,

  /* Production */
  Compat_Reinit_Barometer,
  Compat_Reinit_IMU,

  /* HITL */
  Compat_Disable_IMU,
  Compat_Advance_State,
  Compat_Rewind_State,

  /* Production */
  Compat_Monitor_Altitude,
  Revoke_Monitor_Altitude,

  /* HITL */
  Compat_Consecutive_Samples,
  Revoke_Consecutive_Samples,
  Compat_Reset_Failures,
  Revoke_Reset_Failures,

  /* Production */
  Compat_Validate_Measms,
  Revoke_Validate_Measms,

  /* HITL */
  Compat_Abort_After_40,
  Compat_Abort_After_100,

  /* Production */
  Compat_Abort_After_250,

  /* HITL */
  Compat_Reinit_After_15,
  Compat_Reinit_After_30,
  Compat_Reinit_After_50,

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
    Reinit_IMU,
    Disable_IMU,
    Advance_State,
    Rewind_State,

    Monitor_Altitude,
    revoke(Monitor_Altitude),
    Consecutive_Samples,
    revoke(Consecutive_Samples),
    Reset_Failures,
    revoke(Reset_Failures),
    Validate_Measms,
    revoke(Validate_Measms),

    Abort_After_40,
    Abort_After_100,
    Abort_After_250,
    Reinit_After_15,
    Reinit_After_30,
    Reinit_After_50,
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
  return (enum message)U32(raw[0], raw[1], raw[2], raw[3]);
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

  if (i != cons)
  {
    fetch_add(&gps_mask, 1, Rel);
    idx = i;
  }
}

/*
 * Fetches latest available GPS packet, or checks
 * for timeout. Returns the amount of new entries
 * added since last call. Safe when the calling
 * thread is asynchronously canceled.
 */
static inline fu8
fetch_gps_data(struct coords *buf)
{
  static fu8 idx = UINT_FAST8_MAX;

  fu16 i = idx;
  fu8 n = (fu8)swap(&gps_mask, i << 8, AcqRel);

  if (!n)
  {
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

  if (gps->x > MAX_LAT || gps->x < MIN_LAT)
  {
    st |= Bad_Lattitude;
  }
  if (gps->y > MAX_LON || gps->y < MIN_LON)
  {
    st |= Bad_Longtitude;
  }
  if (gps->z > MAX_SEA || gps->z < MIN_SEA)
  {
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

  if (gps->z > MAX_SEA || gps->z < MIN_SEA)
  {
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
  sweetbench_catch(10);

  timer_update(HeartbeatRF);

  fu32 rep = validate_gps_serial(data, len);

  if (rep != fc_mask(GPS_Data_Code))
  {
    tx_queue_send(&shared, &rep, TX_NO_WAIT);
    return SEDS_ERR;
  }

  fetch_or(&config, option(GPS_Available), Rlx);

  enqueue_gps_data(data);

  sweetbench_start(10, 50);

  return SEDS_OK;
}

/*
 * Converts GPS coordinates into distance from
 * launch rail. This is how Descent KF expects GPS.
 */
static inline void
distance_from_rail(struct coords *gps)
{
  gps->x = fabsf(gps->x - rail.x);
  gps->x = fabsf(gps->y - rail.y);
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
  (void)user;

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
 * Gyroscope sanity check against its data range.
 */
static inline enum message
validate_gyro(const struct coords *gyro, fu32 conf)
{
  enum message st = fc_mask(Sensor_Measm_Code);

  if (conf & option(Validate_Measms))
  {
    if (gyro->x > MAX_DPS || gyro->x < MIN_DPS)
    {
      st |= Bad_Attitude_X;
    }
    if (gyro->y > MAX_DPS || gyro->y < MIN_DPS)
    {
      st |= Bad_Attitude_Y;
    }
    if (gyro->z > MAX_DPS || gyro->z < MIN_DPS)
    {
      st |= Bad_Attitude_Z;
    }
  }

  return st;
}

/*
 * Accelerometer sanity check against its data range.
 */
static inline enum message
validate_accl(const struct coords *accl, fu32 conf)
{
  enum message st = fc_mask(Sensor_Measm_Code);

  if (conf & option(Validate_Measms))
  {
    if (accl->x > MAX_ACC || accl->x < MIN_ACC)
    {
      st |= Bad_Accel_X;
    }
    if (accl->y > MAX_ACC || accl->y < MIN_ACC)
    {
      st |= Bad_Accel_Y;
    }
    if (accl->z > MAX_ACC || accl->z < MIN_ACC)
    {
      st |= Bad_Accel_Z;
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
    if (baro->p > MAX_PRS || baro->p < MIN_PRS)
    {
      st |= Bad_Pressure;
    }
    if (baro->alt > MAX_ALT || baro->alt < MIN_ALT)
    {
      st |= Bad_Altitude;
    }
  }

  return st;
}

/*
 * Call and aggregate statuses from all validator functions.
 */
static inline enum message IREC26_unused
validate_all(const struct measurement *buf, fu32 conf)
{
  return validate_baro(&buf->baro, conf) |
         validate_gyro(&buf->gyro, conf) |
         validate_accl(&buf->d.accl, conf);
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
  fu32 st = 0, counter = 0, conf = 0;

  float accum_baro = 0, accum_gps = 0;
  fu32 ctr_baro = 0, ctr_gps = 0;

  task_loop (conf & option(Launch_Triggered))
  {
    if (fetch_gyro(&payload.gyro))
    {
      st = validate_gyro(&payload.gyro, conf);

      if (st == Sensor_Measm_Code)
      {
        log_measurement(SEDS_DT_GYRO_DATA, &payload.gyro);
      }
      else
      {
        log_err(pilot "malformed Gyro data: %u", st);
      }
    }

    if (fetch_accl(&payload.d.accl))
    {
      st = validate_accl(&payload.d.accl, conf);

      if (st == Sensor_Measm_Code)
      {
        log_measurement(SEDS_DT_ACCEL_DATA, &payload.d.accl);
      }
      else
      {
        log_err(pilot "malformed Accl data: %u", st);
      }
    }

    if (fetch_baro(&payload.baro))
    {
      accum_baro += fsec(timer_exchange(IntervalBaro));
      ++ctr_baro;

      st = validate_baro(&payload.baro, conf);

      if (st == Sensor_Measm_Code)
      {
        log_measurement(SEDS_DT_BAROMETER_DATA, &payload.baro);
      }
      else
      {
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
      rail = payload.d.gps;

      if (counter != 0 &&
          (fabsf(rail.x - payload.d.gps.x) > GPS_RAIL_TOLER ||
           fabsf(rail.y - payload.d.gps.y) > GPS_RAIL_TOLER))
      {
        log_err(id "contraversial launch coords: "
                   "LAT: %f, LON: %f", rail.x, rail.y);
      }
    }

#endif // GPS_AVAILABLE

    if (!(++counter & 255))
    {
      if (ctr_baro > 0)
      {
        log_transition(pi_bar, accum_baro / ctr_baro);
      }
      if (ctr_gps > 0)
      {
        log_transition(pi_gps, accum_gps / ctr_gps);
      }
    }

    tx_thread_relinquish();

    conf = load(&config, Acq);
  }

  /* Request Valve board to ignite the engine.
   */
  task_loop(request_ignition() == SEDS_OK)
    ;

  log_msg(id "ignition requested, in flight mode");
}

/*
 * Data cycle for the Ascent filter.
 */
static inline void ascent_cycle(fu32 conf, fu8 *imu)
{
  fu32 st;
  struct coords suspect;

  sweetbench_start(8);

  if (fetch_gyro(&suspect))
  {
    st = validate_gyro(&suspect, conf);

    if (st == fc_mask(Sensor_Measm_Code))
    {
      payload.gyro = suspect;
      *imu |= Sensor_Gyro;
    }
    else
    {
      tx_queue_send(&shared, &st, TX_NO_WAIT);
    }
  }

  if (fetch_accl(&suspect))
  {
    st = validate_accl(&suspect, conf);

    if (st == fc_mask(Sensor_Measm_Code))
    {
      payload.d.accl = suspect;
      *imu |= Sensor_Accl;
    }
    else
    {
      tx_queue_send(&shared, &st, TX_NO_WAIT);
    }
  }

  if ((*imu & IMU_ID) != IMU_ID)
  {
    if (*imu & ASCENT_PREDICT_DONE)
    {
      goto fast_forward;
    }
    tx_thread_relinquish();
    return;
  }

  *imu &= ~IMU_ID;

  sh.dt = fsec(timer_exchange(AscentKF));

  ascent_predict(sh.dt);
  *imu |= ASCENT_PREDICT_DONE;

  log_measurement(SEDS_DT_GYRO_DATA, &payload.gyro);
  log_measurement(SEDS_DT_ACCEL_DATA, &payload.d.accl);

fast_forward:

  if (!fetch_baro(&payload.baro))
  {
    /* Run Predict in the meanwhile */
    return;
  }

  st = validate_baro(&payload.baro, conf);

  if (st != fc_mask(Sensor_Measm_Code))
  {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
    return;
  }

  *imu &= ~ASCENT_PREDICT_DONE;
  tx_semaphore_put(&eval_focus_mode);

  log_measurement(SEDS_DT_BAROMETER_DATA, &payload.baro);

  sweetbench_catch(8);

  if (conf & Eval_Focus_Flag)
  {
    sweetbench_start(7, 10);
    tx_thread_relinquish();
    sweetbench_catch(7);
  }
}

/*
 * Data cycle for the Descent filter.
 */
static inline void descent_cycle(fu32 conf)
{
  fu32 st = 0;

  sweetbench_start(9);

  sh.dt = fsec(timer_exchange(DescentKF));

  descent_predict(sh.dt);

  if (fetch_baro(&payload.baro))
  {
    st = validate_baro(&payload.baro, conf);

    if (st == fc_mask(Sensor_Measm_Code))
    {
      descent_update(sh.dt);
      st = CAN_EVALUATE;
    }
    else
    {
      tx_queue_send(&shared, &st, TX_NO_WAIT);
    }

    log_measurement(SEDS_DT_BAROMETER_DATA, &payload.baro);
  }

#ifdef GPS_AVAILABLE

  if ((conf & option(GPS_Available)) &&
      fetch_gps_data(&payload.d.gps))
  {
    distance_from_rail(&payload.d.gps);
    descent_update(sh.dt);

    // st = CAN_EVALUATE; // Alex seemed unsure of this.
  }

#endif // GPS_AVAILABLE

  if (st == CAN_EVALUATE)
  {
    evaluate_rocket_state(conf);
    sweetbench_catch(9);
  }
  else
  {
    tx_thread_relinquish();
  }
}

/*
 * Distribution task entry that also serves as
 * an overview of the Flight Computer data distribution.
 * This function is idempotent and can be reentered whenever
 * any critical conditions arise as deemed by Recovery task.
 */
void distribution_entry(ULONG input)
{
  (void)input;

  log_msg(id "started");

  fu32 conf = load(&config, Acq);

  fu8 imu = 0;

  /* Enter pre-launch loop only once.
   */
  if (!(conf & option(Launch_Triggered)))
  {
    pre_launch();
  }

  task_loop(DO_NOT_EXIT)
  {
    conf = load(&config, Acq);

    conf & option(Using_Ascent_KF) ? ascent_cycle(conf, &imu)
                                   : descent_cycle(conf);
  }
}

/*
 * Creates a preemptive, cooperative Distribution
 * task with defined parameters.
 */
UINT create_distribution_task(TX_BYTE_POOL *byte_pool)
{
  UINT st;
  CHAR *pointer;

  /* Allocate the stack for test */
  if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                       DIST_STACK_BYTES, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  st = tx_thread_create(&distribution_task,
                        "Distribution Task",
                        distribution_entry,
                        DIST_INPUT,
                        pointer,
                        DIST_STACK_BYTES,
                        DIST_PRIORITY,
                        /* No preemption threshold */
                        DIST_PRIORITY,
                        DIST_TIME_SLICE,
                        TX_AUTO_START);

  if (st != TX_SUCCESS)
  {
    log_die(id "task creation failure: %u", st);
  }

  return TX_SUCCESS;
}