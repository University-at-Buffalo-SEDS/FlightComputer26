/*
 * Distribution Task
 */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"

#define id "DI "
#define pilot "PI "
#define telid "TE "
#define pi_bar "BAR"
#define pi_gps "GPS"


TX_THREAD distribution_task;

measm meas = {0};

#ifdef GPS_AVAILABLE

static kf_gps rail = {0};
static f_xyz gps_ring[GPS_RING_SIZE] = {0};
static atomic_uint_fast16_t gps_mask = 0xFF00u;

#endif /* GPS_AVAILABLE */


#ifdef TELEMETRY_ENABLED
#ifdef TELEMETRY_CMD_COMPAT

typedef enum remote_cmd_compat : uint8_t
{
  Compat_Deploy_Parachute,
  Compat_Expand_Parachute,
  Compat_Reinit_Sensors,
  Compat_Launch_Signal,
  Compat_Evaluation_Relax,
  Compat_Evaluation_Focus,
  Compat_Evaluation_Abort,
  Compat_Reinit_Barometer,
  Compat_Reinit_IMU,
  Compat_Disable_IMU,
  Compat_Advance_State,
  Compat_Rewind_State,
  Compat_Monitor_Altitude,
  Revoke_Monitor_Altitude,
  Compat_Consecutive_Samples,
  Revoke_Consecutive_Samples,
  Compat_Reset_Failures,
  Revoke_Reset_Failures,
  Compat_Validate_Measms,
  Revoke_Validate_Measms,
  Compat_Abort_After_40,
  Compat_Abort_After_100,
  Compat_Abort_After_250,
  Compat_Reinit_After_15,
  Compat_Reinit_After_30,
  Compat_Reinit_After_50,

  Compat_Messages
} compat;

static const fc_msg extmap[Compat_Messages] = {
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

static inline fc_msg decode_cmd(const uint8_t *raw)
{
  return *raw < Sensors ? extmap[*raw] : Invalid_Message;
}

#else

#define MIN_CMD_SIZE 4

static inline fc_msg decode_cmd(const uint8_t *raw)
{
  return (fc_msg)U32(raw[0], raw[1], raw[2], raw[3]);
}

#endif /* TELEMETRY_CMD_COMPAT */


#ifdef GPS_AVAILABLE

/*
 * Wait-free on atomics, as God intended.
 */
static inline void
enqueue_gps_data(const uint8_t *buf)
{
  static fu8 idx = 0;

  fu8 i = idx;
  memcpy(&gps_ring[i], buf, sizeof(f_xyz));

  i = (i + 1) & GPS_RING_SIZE_MASK;
  fu8 cons = load(&gps_mask, Acq) >> 8;

  if (i != cons)
  {
    fetch_add(&gps_mask, 1, Rel);
    idx = i;
  }
}

/*
 * Transforms GPS data from ring into sub-KF struct.
 */
static inline fu8 fetch_gps_data(kf_gps *buf)
{
  static fu8 idx = UINT_FAST8_MAX;

  fu16 i = idx;
  fu8 n = (fu8)swap(&gps_mask, i << 8, AcqRel);

  if (!n)
  {
    fetch_or(&gps_mask, CLEAR_IDX, Rlx);
    return 0;
  }

  i = (i + n) & GPS_RING_SIZE_MASK;
  idx = i;

  /* Reordering: RF streams XYZ, KF expects [Z]YX */
  buf->sea = gps_ring[i].z;
  buf->lon = gps_ring[i].y;
  buf->lat = gps_ring[i].x;

  fetch_or(&gps_mask, CLEAR_IDX, Rlx);
  return n;
}

/*
 * GPS sanity check against NEO-M9N data range.
 */
static inline fc_msg
validate_gps_absolute(const f_xyz *gps)
{
  fc_msg st = 0;

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
 * GPS sanity check against approximate launch site.
 * Tolerance should be relatively high (> 1 deg).
 */
static inline fc_msg
validate_gps_relative(const f_xyz *gps)
{
  if (!proxim_lat(gps->x) || !proxim_lon(gps->y))
  {
    log_err(telid "GPS too far off site "
                  "LAT: %f, LON: %f", gps->x, gps->y);
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
  fc_msg rep = fc_mask(GPS_Data_Code);

  if (len != sizeof(f_xyz))
  {
    rep |= GPS_Malformed;
  }
  else
  {
    fu32 conf = load(&g_conf, Acq);

    if (conf & option(Validate_Measms))
    {
      rep |= conf & option(Launch_Requested)
                 ? validate_gps_absolute((const f_xyz *)data)
                 : validate_gps_relative((const f_xyz *)data);
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

  fetch_or(&g_conf, option(GPS_Available), Rlx);

  enqueue_gps_data(data);

  sweetbench_start(10, 50);

  return SEDS_OK;
}

/*
 * Converts GPS coordinates into distance from
 * launch rail. This is how Descent KF expects GPS.
 */
static inline void distance_from_rail(kf_gps *buf)
{
  buf->lon = fabsf(buf->lon - rail.lon);
  buf->lat = fabsf(buf->lat - rail.lat);
}

#endif /* GPS_AVAILABLE */


/*
 * Deposits one or multiple messages into the recovery queue.
 */
static inline SedsResult
handle_gnd_command(const uint8_t *data, size_t len)
{
  fc_msg msg;
  UINT st = TX_SUCCESS;

#ifdef MESSAGE_BATCHING_ENABLED

  UINT tlmt_old_pr;

  tx_thread_priority_change(&telemetry_thread, 0,
                                   &tlmt_old_pr);

  for (fu16 k = 0; k < len; k += MIN_CMD_SIZE)
  {
    msg = decode_cmd(data + k);

    st += (msg != Invalid_Message)
              ? tx_queue_send(&shared, &msg, TX_NO_WAIT)
              : INVALID_MESSAGE_STATUS;
  }

  tx_thread_priority_change(&telemetry_thread,
                            TLMT_PRIORITY, &tlmt_old_pr);

#else

  msg = decode_cmd(data);

  st = (msg != Invalid_Message)
            ? tx_queue_send(&shared, &msg, TX_NO_WAIT)
            : INVALID_MESSAGE_STATUS;

#endif /* MESSAGE_BATCHING_ENABLED */

  return st == TX_SUCCESS ? SEDS_OK : SEDS_ERR;
}

/*
 * Invokes an appropriate handler based on sender id.
 */
SedsResult
on_fc_packet(const SedsPacketView *pkt, void *user)
{
  (void)user;

  if (!pkt || pkt->ty != SEDS_EP_FLIGHT_CONTROLLER ||
      !pkt->payload || !pkt->payload_len ||
      !pkt->sender || !pkt->sender_len)
  {
    return SEDS_HANDLER_ERROR;
  }

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

  else return SEDS_HANDLER_ERROR;
}

#endif /* TELEMETRY_ENABLED */


/*
 * Gyroscope sanity check against its data range.
 */
static inline fc_msg
validate_gyro(const f_xyz *gyro, fu32 conf)
{
  fc_msg st = fc_mask(Sensor_Measm_Code);

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
static inline fc_msg
validate_accl(const f_xyz *accl, fu32 conf)
{
  fc_msg st = fc_mask(Sensor_Measm_Code);

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
static inline fc_msg
validate_baro(const baro *baro, fu32 conf)
{
  fc_msg st = fc_mask(Sensor_Measm_Code);

  if (conf & option(Validate_Measms))
  {
    if (baro->prs > MAX_PRS || baro->prs < MIN_PRS)
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
 * Calls and aggregate statuses from all validators.
 */
static inline fc_msg IREC26_unused
validate_all(const measm *buf, fu32 conf)
{
  return validate_baro(&buf->baro, conf) |
         validate_gyro(&buf->gyro, conf) |
         validate_accl(&buf->accl, conf);
}


/*
 * Monitors and reports GPS throughout fill sequence.
 */
static inline void monitor_gps(fu32 conf, float *acc)
{
#ifdef GPS_AVAILABLE

  if (conf & option(GPS_Available) &&
      fetch_gps_data(&meas.gps))
  {
    *acc += fsec(timer_exchange(FillSequence));

    rail = meas.gps;

    if (!within(rail.lat - meas.gps.lat, GPS_RAIL_TOLER) ||
        !within(rail.lon - meas.gps.lon, GPS_RAIL_TOLER))
    {
      log_err(id "new GPS reference "
                 "LAT: %f, LON: %f", rail.lat, rail.lon);
    }
  }

#endif /* GPS_AVAILABLE */
}

/*
 * Stage 0 of fill sequence: FC streams data
 * for human inspection and accepts runtime config options.
 */
static inline void data_streaming_mode(void)
{
  fu32 ctr = 0, conf = 0;

  float acc_baro = 0.0f, acc_gps = 0.0f;
  fu32 ctr_baro = 0, ctr_gps = 0;

  task_loop (conf & option(Postinit_Requested) ||
             conf & option(Rollback_Requested))
  {
    fu32 st = fc_mask(Sensor_Measm_Code);

    if (fetch_gyro(&meas.gyro))
    {
      st |= validate_gyro(&meas.gyro, conf);
      log_measm(SEDS_DT_GYRO_DATA, &meas.gyro);
    }

    if (fetch_accl(&meas.accl))
    {
      st |= validate_accl(&meas.accl, conf);
      log_measm(SEDS_DT_ACCEL_DATA, &meas.accl);
    }

    if (fetch_baro(&meas.baro))
    {
      acc_baro += fsec(timer_exchange(Auxiliary));
      ++ctr_baro;

      st |= validate_baro(&meas.baro, conf);
      log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);
    }

    if (st != fc_mask(Sensor_Measm_Code))
    {
      log_err(pilot "malformed measm: %u", fc_unmask(st));
    }

    monitor_gps(conf, &acc_gps);

    if (!(++ctr & 255))
    {
      if (ctr_baro > 0)
      {
        log_transition(pi_bar, acc_baro / ctr_baro);
      }
      if (ctr_gps > 0)
      {
        log_transition(pi_gps, acc_gps / ctr_gps);
      }
    }

    tx_thread_relinquish();

    conf = load(&g_conf, Acq);
  }
}

/*
 * Stage 1 of fill sequence: euler angles -> quaternions.
 */
static inline void post_initialization(void)
{
  f_xyz accl_acc = {0};
  float acc_gps = 0.0f;
  fu32 ctr_gps = 0, ctr_accl = 0;

  fu32 st, conf = load(&g_conf, Acq);

  timer_update(Auxiliary);

  task_loop (timer_fetch(Auxiliary) > POSTINIT_DURATION)
  {
    if (fetch_accl(&meas.accl))
    {
      st = validate_accl(&meas.accl, conf);
      log_measm(SEDS_DT_ACCEL_DATA, &meas.accl);

      if (st == fc_mask(Sensor_Measm_Code))
      {
        accl_acc.x += meas.accl.x;
        accl_acc.y += meas.accl.y;
        accl_acc.z += meas.accl.z;
        ++ctr_accl;
      }
    }

    monitor_gps(conf, &acc_gps);

    tx_thread_relinquish();

    conf = load(&g_conf, Acq);
  }

  if (ctr_gps > 0)
  {
    log_transition(pi_gps, acc_gps / ctr_gps);
  }

  accl_acc.x /= ctr_accl;
  accl_acc.y /= ctr_accl;
  accl_acc.z /= ctr_accl;

  accel_to_quaternion(&accl_acc);
}

/*
 * Data cycle for the Ascent filter: Update stage.
 */
static inline void ascent_cycle_update(fu32 conf, fu8 *imu)
{
  fu32 st;

  if (!fetch_baro(&meas.baro))
  {
    return;
  }

  st = validate_baro(&meas.baro, conf);

  if (st != fc_mask(Sensor_Measm_Code))
  {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
    return;
  }

  *imu &= ~ASCENT_PREDICT_DONE;
  tx_semaphore_put(&eval_focus_mode);

  log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);

  sweetbench_catch(8);

  conf = fetch_and(&g_conf, ~option(Ascent_Finished), Rel);

  if (conf & Eval_Focus_Flag)
  {
    sweetbench_start(7, 10);
    tx_thread_relinquish();
    sweetbench_catch(7);
  }
}

/*
 * Data cycle for the Ascent filter: Predict stage.
 */
static inline void ascent_cycle(fu32 conf, fu8 *imu)
{
  fu32 st;
  f_xyz suspect;

  sweetbench_start(8);

  if (fetch_gyro(&suspect))
  {
    st = validate_gyro(&suspect, conf);

    if (st == fc_mask(Sensor_Measm_Code))
    {
      meas.gyro = suspect;
      *imu |= Sensor_Gyro;
    }
    else tx_queue_send(&shared, &st, TX_NO_WAIT);
  }

  if (fetch_accl(&suspect))
  {
    st = validate_accl(&suspect, conf);

    if (st == fc_mask(Sensor_Measm_Code))
    {
      meas.accl = suspect;
      *imu |= Sensor_Accl;
    }
    else tx_queue_send(&shared, &st, TX_NO_WAIT);
  }

  if ((*imu & IMU_ID) != IMU_ID)
  {
    if (*imu & ASCENT_PREDICT_DONE)
    {
      ascent_cycle_update(conf, imu);
    }
    else tx_thread_relinquish();

    return;
  }
  else if (!(conf & option(Ascent_Finished)))
  {
    return;
  }

  *imu &= ~IMU_ID;

  sm.dt = fsec(timer_exchange(AscentKF));

  ascent_predict(sm.dt);
  *imu |= ASCENT_PREDICT_DONE;

  log_measm(SEDS_DT_GYRO_DATA, &meas.gyro);
  log_measm(SEDS_DT_ACCEL_DATA, &meas.accl);

  ascent_cycle_update(conf, imu);
}

/*
 * Data cycle for the Descent filter.
 */
static inline void descent_cycle(fu32 conf)
{
  fu32 st = 0;

  sweetbench_start(9);

  sm.dt = fsec(timer_exchange(DescentKF));

  descent_predict(sm.dt);

  if (fetch_baro(&meas.baro))
  {
    st = validate_baro(&meas.baro, conf);

    if (st == fc_mask(Sensor_Measm_Code))
    {
      descent_update(sm.dt);
      st = CAN_EVALUATE;
    }
    else tx_queue_send(&shared, &st, TX_NO_WAIT);

    log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);
  }

#ifdef GPS_AVAILABLE

  if ((conf & option(GPS_Available)) &&
      fetch_gps_data(&meas.gps))
  {
    distance_from_rail(&meas.gps);
    descent_update(sm.dt);

    if (!(conf & option(Monitor_Altitude)))
    {
      st = CAN_EVALUATE;
    }
  }

#endif /* GPS_AVAILABLE */

  if (st == CAN_EVALUATE)
  {
    evaluate_rocket_state(conf);
    sweetbench_catch(9);
  }
  else tx_thread_relinquish();
}


/*
 * Operates flight states before launch, runs KF
 * data distribution loops.
 */
void distribution_entry(ULONG input)
{
  (void)input;

  fu8 imu = 0;
  fu32 conf = load(&g_conf, Acq);

  /* Fill sequence is here
   */
  if (!(conf & option(Launch_Requested)))
  {
    data_streaming_mode();

    conf = load(&g_conf, Acq);
    check_rollback_request(conf);

    post_initialization();

    conf = load(&g_conf, Acq);
    check_rollback_request(conf);

    log_msg(id "postinit done, awaiting launch signal");

    task_loop (conf & option(Launch_Requested))
    {
      tx_thread_sleep(POSTINIT_INTERVAL);
      conf = load(&g_conf, Acq);
      check_rollback_request(conf);
    }

    task_loop (request_ignition() == SEDS_OK)
      ;
    log_msg(id "ignition requested, in flight mode");
  }

  task_loop (DO_NOT_EXIT)
  {
    conf = load(&g_conf, Acq);
    check_rollback_request(conf);

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