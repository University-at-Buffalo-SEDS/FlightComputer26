/* Core/Src/distribution.c */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"

#define id "DI "


TX_THREAD distribution_task;

measm meas = {0};

spinlock meas_locks[MEMS_Devices] = {0};

#ifdef GPS_AVAILABLE

static kf_gps rail = {0};
static f_xyz gps_ring[GPS_RING_SIZE] = {0};
static atomic_uint_fast16_t gps_mask = 0xFF00u;

#endif /* GPS_AVAILABLE */


#ifdef TELEMETRY_ENABLED
#ifdef TELEMETRY_CMD_COMPAT

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
    Measm_Reports,
    revoke(Measm_Reports),

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

static inline fc_msg decode_cmd(const uint8_t *k)
{
  return *k < Compat_Messages ? extmap[*k] : Invalid_Message;
}

#else /* TELEMETRY_CMD_COMPAT */

#define MIN_CMD_SIZE 4

static inline fc_msg decode_cmd(const uint8_t *k)
{
  return (fc_msg) fc_unmask(U32(k[0], k[1], k[2], k[3]));
}

#endif /* TELEMETRY_CMD_COMPAT */

/* GPS coordinates handling */

static inline void enqueue_raw_coords(const uint8_t *buf)
{
#ifdef GPS_AVAILABLE

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

#endif /* GPS_AVAILABLE */
}

#endif /* TELEMETRY_ENABLED */

static inline fu8 fetch_gps_data(kf_gps *buf)
{
#ifdef GPS_AVAILABLE

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

#else
  return 0;

#endif /* GPS_AVAILABLE */
}

#ifdef TELEMETRY_ENABLED

static inline fu32
validate_coords(const f_xyz *gps, size_t len, fu32 conf)
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

static inline SedsResult
process_gps_packet(const uint8_t *data, size_t len)
{
#ifdef GPS_AVAILABLE

  sweetbench_catch(10);

  timer_update(HeartbeatRF);

  fu32 cfg = fetch_or(&g_conf, option(GPS_Available), Rlx);
  fu32 rep = validate_coords((const f_xyz *)data, len, cfg);

  if (rep != fc_mask(GPS_Data_Code))
  {
    tx_queue_send(&shared, &rep, TX_NO_WAIT);
    return SEDS_ERR;
  }

  enqueue_raw_coords(data);

  sweetbench_start(10, 50);

#endif /* GPS_AVAILABLE */

  return SEDS_OK;
}

#endif /* TELEMETRY_ENABLED */

static inline void to_relative_coords(kf_gps *buf)
{
#ifdef GPS_AVAILABLE
  buf->lon = fabsf(buf->lon - rail.lon);
  buf->lat = fabsf(buf->lat - rail.lat);
#endif
}

static inline fu32
watch_for_gps_packets(fu32 conf, float *acc, fu32 *ctr)
{
#ifdef GPS_AVAILABLE

  if (conf & option(GPS_Available) &&
      fetch_gps_data(&meas.gps))
  {
    *acc += fsec(timer_exchange(GPSWatchdog));

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


/* General packet handling */

#ifdef TELEMETRY_ENABLED

static inline SedsResult
dispatch_flight_cmd(const uint8_t *data, size_t len)
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

#else /* MESSAGE_BATCHING_ENABLED */

  msg = decode_cmd(data);

  st = msg != Invalid_Message
            ? tx_queue_send(&shared, &msg, TX_NO_WAIT)
            : INVALID_MESSAGE_STATUS;

#endif /* MESSAGE_BATCHING_ENABLED */

  return st == TX_SUCCESS ? SEDS_OK : SEDS_ERR;
}

static inline SedsResult pulse_ground(void)
{
  sweetbench_catch(11);
  timer_update(HeartbeatGND);
  sweetbench_start(11, 100);
  return SEDS_OK;
}

static inline SedsResult
update_ascent_biases(const uint8_t *data, size_t len)
{
  if (len != sizeof(ekf_bias))
  {
    return SEDS_ERR;
  }

  if (load(&g_conf, Acq) & option(Launch_Requested))
  {
    log_err(id "biases rejected mid-flight");
    return SEDS_ERR;
  }

  /* TODO Bias validation? */

  for (fu8 k = 0; k < STATE_HISTORY; ++k)
  {
    svec(k).bias = *(ekf_bias *)data;
  }

  imedsv.bias = *(ekf_bias *)data;

  fetch_or(&g_conf, option(Manual_Biases), Rel);

  return SEDS_OK;
}

SedsResult on_fc_packet(const SedsPacketView *pkt, void *_)
{
  if (!pkt || !pkt->sender || !pkt->sender_len ||
      !pkt->payload || !pkt->payload_len)
  {
    return SEDS_HANDLER_ERROR;
  }

  switch (pkt->ty)
  {
    case SEDS_DT_HEARTBEAT:
      return pulse_ground();
    case SEDS_DT_GPS_DATA:
      return process_gps_packet(pkt->payload, pkt->payload_len);
    case SEDS_DT_FLIGHT_COMMAND:
      return dispatch_flight_cmd(pkt->payload, pkt->payload_len);
    case SEDS_DT_ASCENT_BIASES:
      return update_ascent_biases(pkt->payload, pkt->payload_len);
  }

  return SEDS_HANDLER_ERROR;
}

#endif /* TELEMETRY_ENABLED */


/* On-board sensor data validation */

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

  if (conf & option(Measm_Reports) &&
      (conf & option(Reset_Failures) ||
       st != fc_mask(Sensor_Measm_Code)))
  {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
  }

  return st == fc_mask(Sensor_Measm_Code);
}

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

  if (conf & option(Measm_Reports) &&
      (conf & option(Reset_Failures) ||
       st != fc_mask(Sensor_Measm_Code)))
  {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
  }

  return st == fc_mask(Sensor_Measm_Code);
}

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

  if (conf & option(Measm_Reports) &&
      (conf & option(Reset_Failures) ||
       st != fc_mask(Sensor_Measm_Code)))
  {
    tx_queue_send(&shared, &st, TX_NO_WAIT);
  }

  return st == fc_mask(Sensor_Measm_Code);
}

static inline conditional fu8
validate_all(const measm *buf, fu32 conf)
{
  return validate_baro(&buf->baro, conf) +
         validate_gyro(&buf->gyro, conf) +
         validate_accl(&buf->accl, conf);
}


/* Stage 0 of fill sequence */

static inline void data_streaming_mode(void)
{
  float acc_baro = 0.0f, acc_gps = 0.0f;
  fu32 ctr_baro = 0, ctr_gps = 0, conf = 0;

  task_loop (conf & option(Postinit_Requested) ||
             conf & option(Rollback_Requested))
  {
    fu32 code = 0;

    if (try_fetch_gyro(&meas.gyro))
    {
      code |= validate_gyro(&meas.gyro, conf) ? 0
                                              : Gyro_Mask;
      log_measm(SEDS_DT_GYRO_DATA, &meas.gyro);
    }

    if (try_fetch_accl(&meas.accl))
    {
      code |= validate_accl(&meas.accl, conf) ? 0
                                              : Accl_Mask;
      log_measm(SEDS_DT_ACCEL_DATA, &meas.accl);
    }

    if (try_fetch_baro(&meas.baro))
    {
      acc_baro += fsec(timer_exchange(Auxiliary));
      log_metric(id "Baro interval", acc_baro / ++ctr_baro, false);

      code |= validate_baro(&meas.baro, conf) ? 0
                                              : Baro_Mask;
      log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);
    }

    if (code != 0)
    {
      log_err(id "malformed measm: %u", code);
    }

    if (watch_for_gps_packets(conf, &acc_gps, &ctr_gps) > 0)
    {
      log_metric(id "GPS interval", acc_gps / ctr_gps, false);
    }

    tx_thread_relinquish();

    conf = load(&g_conf, Acq);
  }
}


/* Stage 1 of fill sequence */

static inline void post_initialization(void)
{
  f_xyz accl_acc = {0};
  float acc_gps = 0.0f;
  fu32 ctr_gps = 0, ctr_accl = 0;

  fc_msg cmd = option(Reinit_Sensors);
  tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);

  fu32 conf = load(&g_conf, Acq);

  timer_update(Auxiliary);

  task_loop (timer_fetch(Auxiliary) > POSTINIT_DURATION)
  {
    if (try_fetch_accl(&meas.accl))
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

    (void) watch_for_gps_packets(conf, &acc_gps, &ctr_gps);

    tx_thread_relinquish();

    conf = load(&g_conf, Acq);
  }

  if (ctr_gps > 0)
  {
    log_metric(id "Avg GPS (sec)", acc_gps / ctr_gps, true);
  }

  accl_acc.x /= ctr_accl;
  accl_acc.y /= ctr_accl;
  accl_acc.z /= ctr_accl;

  accel_to_quaternion(&accl_acc);
}


/* Distribution for Ascent */

static inline void for_ascent_update(fu32 conf)
{
  baro baro_suspect;

  if (try_fetch_baro(&baro_suspect) &&
      validate_baro(&baro_suspect, conf))
  {
    fc_lock(&meas_locks[Baro]);
    meas.baro = baro_suspect;
    fc_concede(&meas_locks[Baro]);
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

static inline void for_ascent_predict(fu32 conf, fu8 *imu)
{
  static f_xyz accum_gyro, accum_accl;

  f_xyz suspect_gyro, suspect_accl;

  sweetbench_start(8);

  if (try_fetch_gyro(&suspect_gyro) && 
      validate_gyro(&suspect_gyro, conf))
  {
    accum_gyro = suspect_gyro;
    *imu |= Gyro_Mask;
  }

  if (try_fetch_accl(&suspect_accl) &&
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

  fc_lock(&meas_locks[IMU]);

  meas.gyro = accum_gyro;
  meas.accl = accum_accl;

  fc_concede(&meas_locks[IMU]);

  tx_event_flags_set(&eval_stage, Gyro_Mask | Accl_Mask, TX_OR);

  *imu &= ~(Gyro_Mask | Accl_Mask);

  log_measm(SEDS_DT_GYRO_DATA, &suspect_gyro);
  log_measm(SEDS_DT_ACCEL_DATA, &suspect_accl);

  if (conf & option(Eval_Focus_Flag))
  {
    tx_thread_relinquish();
  }
}


/* Distribution for Descent and itself */

static inline void descent_full_cycle(fu32 conf)
{
  fu32 stage = 0;

  sweetbench_start(9);

  const float dt = fsec(timer_exchange(DescentKF));

  descent_predict(dt);

  if (try_fetch_baro(&meas.baro) &&
      validate_baro(&meas.baro, conf))
  {
    descent_update();
    stage = EVALUATION_STAGED;
    log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);
  }

  if ((conf & option(GPS_Available)) &&
      fetch_gps_data(&meas.gps))
  {
    to_relative_coords(&meas.gps);
    descent_update();

    if (!(conf & option(Monitor_Altitude)))
    {
      stage = EVALUATION_STAGED;
    }
  }

  if (stage == EVALUATION_STAGED)
  {
    evaluate_rocket_state(conf);
    sweetbench_catch(9);
  }
  else tx_thread_relinquish();
}


/* Task */

void distribution_entry(ULONG _)
{
  fu8 imu = 0;
  fu32 conf = load(&g_conf, Acq);

  if (!(conf & option(Launch_Requested)))
  {
    data_streaming_mode();

    conf = load(&g_conf, Acq);
    check_rollback_request(conf);

    /* Assert: state is Postinit */
    log_flight_state(to_global_state(current()));
    post_initialization();

    conf = load(&g_conf, Acq);
    check_rollback_request(conf);

    log_critical(id "complete, awaiting launch signal");

    task_loop (conf & option(Launch_Requested))
    {
      tx_thread_sleep(POSTINIT_INTERVAL);
      conf = load(&g_conf, Acq);
      check_rollback_request(conf);
    }

    task_loop (request_ignition() == SEDS_OK)
      ;
    log_critical(id "ignition requested, in flight mode");
  }

  task_loop (DO_NOT_EXIT)
  {
    conf = load(&g_conf, Acq);
    check_rollback_request(conf);

    if (conf & option(Using_Ascent_KF))
    {
      conf & option(Ascent_KF_Staged)
           ? for_ascent_update(conf)
           : for_ascent_predict(conf, &imu);
    }
    else descent_full_cycle(conf);
  }
}

UINT create_distribution_task(TX_BYTE_POOL *byte_pool)
{
  UINT st;
  CHAR *pointer;

  const char *critical = "creation failure:";

  st = tx_byte_allocate(byte_pool, (VOID **)&pointer,
                        DIST_STACK_BYTES, TX_NO_WAIT);

  if (st != TX_SUCCESS)
  {
    log_die(id "stack %s %u", critical, st);
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
    log_die(id "task %s %u", critical, st);
  }

  return TX_SUCCESS;
}