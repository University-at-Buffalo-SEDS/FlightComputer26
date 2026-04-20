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

atomic_uint_fast8_t meas_locks[Sensors - 1] = {0};

#ifdef GPS_AVAILABLE

static kf_gps rail = {0};
static f_xyz gps_ring[GPS_RING_SIZE] = {0};
static atomic_uint_fast16_t gps_mask = 0xFF00u;

#endif /* GPS_AVAILABLE */


#ifdef TELEMETRY_ENABLED
#ifdef TELEMETRY_CMD_COMPAT

typedef enum remote_cmd_compat : uint8_t
{
  Compat_Postinit_Signal,
  Compat_Launch_Signal,
  Compat_Rollback_Signal,

  Compat_Monitor_Altitude,
  Revoke_Monitor_Altitude,
  Compat_Consecutive_Samples,
  Revoke_Consecutive_Samples,
  Compat_Reset_Failures,
  Revoke_Reset_Failures,
  Compat_Report_Bad_Measms,
  Revoke_Report_Bad_Measms,

  Compat_Deploy_Parachute,
  Compat_Expand_Parachute,
  Compat_Evaluation_Relax,
  Compat_Evaluation_Focus,
  Compat_Evaluation_Abort,
  Compat_Reinit_Sensors,
  Compat_Reinit_Barometer,
  Compat_Reinit_IMU,
  Compat_Disable_IMU,
  Compat_Advance_State,
  Compat_Rewind_State,

  Compat_Abort_After_40,
  Compat_Abort_After_100,
  Compat_Abort_After_250,
  Compat_Reinit_After_15,
  Compat_Reinit_After_30,
  Compat_Reinit_After_50,

  Compat_Messages
} compat;

static const fc_msg extmap[Compat_Messages] = {
    Postinit_Signal,
    Launch_Signal,
    Rollback_Signal,

    Monitor_Altitude,
    revoke(Monitor_Altitude),
    Consecutive_Samples,
    revoke(Consecutive_Samples),
    Reset_Failures,
    revoke(Reset_Failures),
    Report_Bad_Measms,
    revoke(Report_Bad_Measms),

    Deploy_Parachute,
    Expand_Parachute,
    Evaluation_Relax,
    Evaluation_Focus,
    Evaluation_Abort,
    Reinit_Sensors,
    Reinit_Barometer,
    Reinit_IMU,
    Disable_IMU,
    Advance_State,
    Rewind_State,

    Abortion_Thresholds + 40,
    Abortion_Thresholds + 100,
    Abortion_Thresholds + 250,
    Reinit_Thresholds + 15,
    Reinit_Thresholds + 30,
    Reinit_Thresholds + 50,
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
  return (fc_msg) fc_unmask(U32(raw[0], raw[1], raw[2], raw[3]));
}

#endif /* TELEMETRY_CMD_COMPAT */

#endif /* TELEMETRY_ENABLED */

#ifdef GPS_AVAILABLE

/*
 * Wait-free on atomics, as God intended.
 */
static inline void
enqueue_gps_data(const uint8_t *buf)
{
  static fu8 idx = 0;

  fu8 i = idx;
  gps_ring[i] = *(f_xyz *)buf;

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
  fu8 n = (fu8) swap(&gps_mask, i << 8, AcqRel);

  if (n == 0)
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
 * Checks received size and applicable sanity constraints.
 */
static inline fu32
validate_gps(const f_xyz *gps, size_t len, fu32 conf)
{
  if (len != sizeof(f_xyz))
  {
    return fc_mask(GPS_Data_Code | GPS_Malformed);
  }

  fc_msg st = fc_mask(GPS_Data_Code);
  bool launch = conf & option(Launch_Requested);
  
  if (launch ? !proxim_lat(gps->x)
             : gps->x > MAX_LAT || gps->x < MIN_LAT)
  {
    st |= Bad_Lattitude;
  }
  if (launch ? !proxim_lon(gps->y)
             : gps->y > MAX_LON || gps->y < MIN_LON)
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
 * Accumulates helpers refresh RF board heartbeat,
 * validate GPS data, and enqueue valid packet.
 */
static inline SedsResult
handle_gps_data(const uint8_t *data, size_t len)
{
#ifdef GPS_AVAILABLE

  sweetbench_catch(10);

  timer_update(HeartbeatRF);

  fu32 cfg = fetch_or(&g_conf, option(GPS_Available), Rlx);
  fu32 rep = validate_gps((const f_xyz *)data, len, cfg);

  if (rep != fc_mask(GPS_Data_Code))
  {
    tx_queue_send(&shared, &rep, TX_NO_WAIT);
    return SEDS_ERR;
  }

  enqueue_gps_data(data);

  sweetbench_start(10, 50);

#endif /* GPS_AVAILABLE */

  return SEDS_OK;
}

/*
 * Converts GPS coordinates into distance from launch rail.
 */
static inline void distance_from_rail(kf_gps *buf)
{
  buf->lon = fabsf(buf->lon - rail.lon);
  buf->lat = fabsf(buf->lat - rail.lat);
}

#endif /* GPS_AVAILABLE */


#ifdef TELEMETRY_ENABLED

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

    st += msg != Invalid_Message
              ? tx_queue_send(&shared, &msg, TX_NO_WAIT)
              : INVALID_MESSAGE_STATUS;
  }

  tx_thread_priority_change(&telemetry_thread,
                            TLMT_PRIORITY, &tlmt_old_pr);

#else

  msg = decode_cmd(data);

  st = msg != Invalid_Message
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
  else if (pkt->sender[0] == 'R')
  {
    return handle_gps_data(pkt->payload, pkt->payload_len);
  }
  else return SEDS_HANDLER_ERROR;
}

#endif /* TELEMETRY_ENABLED */


/*
 * Gyroscope sanity check against its data range.
 */
static inline bool
validate_gyro(const f_xyz *gyro, fu32 conf)
{
  fc_msg st = fc_mask(Sensor_Measm_Code);

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

  if (conf & option(Report_Bad_Measms) &&
      st != fc_mask(Sensor_Measm_Code))
  {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
    return false;
  }

  return true;
}

/*
 * Accelerometer sanity check against its data range.
 */
static inline bool
validate_accl(const f_xyz *accl, fu32 conf)
{
  fc_msg st = fc_mask(Sensor_Measm_Code);

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

  if (conf & option(Report_Bad_Measms) &&
      st != fc_mask(Sensor_Measm_Code))
  {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
    return false;
  }

  return true;
}

/*
 * Barometer sanity check against BMP390 data range.
 */
static inline bool
validate_baro(const baro *baro, fu32 conf)
{
  fc_msg st = fc_mask(Sensor_Measm_Code);

  if (baro->prs > MAX_PRS || baro->prs < MIN_PRS)
  {
    st |= Bad_Pressure;
  }
  if (baro->alt > MAX_ALT || baro->alt < MIN_ALT)
  {
    st |= Bad_Altitude;
  }

  if (conf & option(Report_Bad_Measms) &&
      st != fc_mask(Sensor_Measm_Code))
  {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
    return false;
  }

  return true;
}

/*
 * Calls and aggregate statuses from all validators.
 */
static inline fu8 IREC26_unused
validate_all(const measm *buf, fu32 conf)
{
  return validate_baro(&buf->baro, conf) +
         validate_gyro(&buf->gyro, conf) +
         validate_accl(&buf->accl, conf);
}


/*
 * Monitors and reports GPS throughout fill sequence.
 */
static inline fu32 monitor_gps(fu32 conf, float *acc, fu32 *ctr)
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

  return ++*ctr;

#else
  return 0;

#endif /* GPS_AVAILABLE */
}

/*
 * Stage 0 of fill sequence: FC streams data
 * for human inspection and accepts runtime config options.
 */
static inline void data_streaming_mode(void)
{
  float acc_baro = 0.0f, acc_gps = 0.0f;
  fu32 ctr_baro = 0, ctr_gps = 0, conf = 0;

  task_loop (conf & option(Postinit_Requested) ||
             conf & option(Rollback_Requested))
  {
    fu32 code = 0;

    if (fetch_gyro(&meas.gyro))
    {
      code |= validate_gyro(&meas.gyro, conf) ? 0
                                              : Gyro_Mask;
      log_measm(SEDS_DT_GYRO_DATA, &meas.gyro);
    }

    if (fetch_accl(&meas.accl))
    {
      code |= validate_accl(&meas.accl, conf) ? 0
                                              : Accl_Mask;
      log_measm(SEDS_DT_ACCEL_DATA, &meas.accl);
    }

    if (fetch_baro(&meas.baro))
    {
      acc_baro += fsec(timer_exchange(Auxiliary));
      log_transition(pi_bar, acc_baro / ++ctr_baro);

      code |= validate_baro(&meas.baro, conf) ? 0
                                              : Baro_Mask;
      log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);
    }

    if (code != 0)
    {
      log_err(pilot "malformed measm: %u", code);
    }

    if (monitor_gps(conf, &acc_gps, &ctr_gps) > 0)
    {
      log_transition(pi_gps, acc_gps / ctr_gps);
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

  fc_msg cmd = option(Reinit_IMU);
  tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);

  fu32 conf = load(&g_conf, Acq);

  timer_update(Auxiliary);

  task_loop (timer_fetch(Auxiliary) > POSTINIT_DURATION)
  {
    if (fetch_accl(&meas.accl))
    {
      log_measm(SEDS_DT_ACCEL_DATA, &meas.accl);

      if (validate_accl(&meas.accl, conf))
      {
        accl_acc.x += meas.accl.x;
        accl_acc.y += meas.accl.y;
        accl_acc.z += meas.accl.z;
        ++ctr_accl;
      }
    }

    (void) monitor_gps(conf, &acc_gps, &ctr_gps);

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
static inline void asc_upd(fu32 conf)
{
  baro baro_suspect;

  if (fetch_baro(&baro_suspect) &&
      validate_baro(&baro_suspect, conf))
  {
    fc_lock(&meas_locks[1]);
    meas.baro = baro_suspect;
    fc_unlock(&meas_locks[1]);
  }
  else return;

  tx_event_flags_set(&eval_stage, Baro_Mask, TX_OR);

  log_measm(SEDS_DT_BAROMETER_DATA, &baro_suspect);

  sweetbench_catch(8);

  if (conf & Eval_Focus_Flag)
  {
    tx_thread_relinquish();
  }
}

/*
 * Data cycle for the Ascent filter: Predict stage.
 */
static inline void asc_pred(fu32 conf, fu8 *imu)
{
  static f_xyz accum_gyro, accum_accl;

  f_xyz suspect_gyro, suspect_accl;

  sweetbench_start(8);

  if (fetch_gyro(&suspect_gyro) && 
      validate_gyro(&suspect_gyro, conf))
  {
    accum_gyro = suspect_gyro;
    *imu |= Gyro_Mask;
  }

  if (fetch_accl(&suspect_accl) &&
      validate_accl(&suspect_accl, conf))
  {
    accum_accl = suspect_accl;
    *imu |= Accl_Mask;
  }

  if (*imu != (Gyro_Mask | Accl_Mask))
  {
    tx_thread_relinquish();
    return;
  }

  fc_lock(&meas_locks[0]);

  meas.gyro = accum_gyro;
  meas.accl = accum_accl;

  fc_unlock(&meas_locks[0]);

  tx_event_flags_set(&eval_stage, Gyro_Mask | Accl_Mask, TX_OR);

  *imu &= ~(Gyro_Mask | Accl_Mask);

  log_measm(SEDS_DT_GYRO_DATA, &suspect_gyro);
  log_measm(SEDS_DT_ACCEL_DATA, &suspect_accl);

  if (conf & option(Eval_Focus_Flag))
  {
    tx_thread_relinquish();
  }
}

/*
 * Data cycle for the Descent filter.
 */
static inline void desc_full_cycle(fu32 conf)
{
  fu32 stage = 0;

  sweetbench_start(9);

  const float dt = fsec(timer_exchange(DescentKF));

  descent_predict(dt);

  if (fetch_baro(&meas.baro) &&
      validate_baro(&meas.baro, conf))
  {
    descent_update();
    stage = EVALUATION_STAGED;
    log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);
  }

#ifdef GPS_AVAILABLE

  if ((conf & option(GPS_Available)) &&
      fetch_gps_data(&meas.gps))
  {
    distance_from_rail(&meas.gps);
    descent_update();

    if (!(conf & option(Monitor_Altitude)))
    {
      stage = EVALUATION_STAGED;
    }
  }

#endif /* GPS_AVAILABLE */

  if (stage == EVALUATION_STAGED)
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

    if (conf & option(Using_Ascent_KF))
    {
      conf & option(Ascent_KF_Staged) ? asc_upd(conf)
                                      : asc_pred(conf, &imu);
    }
    else desc_full_cycle(conf);
  }
}

/*
 * Creates a preemptive, cooperative Distribution task
 * with defined parameters.
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