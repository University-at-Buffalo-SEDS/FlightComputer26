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
static rf_receiver rfboard = {0};
#endif


#if defined (TELEMETRY_ENABLED) || defined(FAKESTATION)
#ifdef TELEMETRY_CMD_COMPAT

static const fc_msg extmap[Compat_Messages] = {
    Postinit_Signal,
    Launch_Signal,

    Vigilant_Mode,
    revoke(Vigilant_Mode),
    Eval_Successive,
    revoke(Eval_Successive),
    Reset_Failures,
    revoke(Reset_Failures),
    Measm_Reports,
    revoke(Measm_Reports),
    Velocity_Checks,
    revoke(Velocity_Checks),

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
#endif /* TELEMETRY_ENABLED + FAKESTATION */


/* GPS coordinates handling */

static inline bool fetch_gps_data(kf_gps *buf)
{
#ifdef GPS_AVAILABLE

  fc_lock(&rfboard.rflock);
  
  if (!rfboard.updated)
  {
    fc_unlock(&rfboard.rflock);
    return false;
  }

  /* Reordering: RF streams XYZ, KF expects [Z]YX */
  buf->sea = rfboard.coords_buf.z;
  buf->lon = rfboard.coords_buf.y;
  buf->lat = rfboard.coords_buf.x;

  fc_unlock(&rfboard.rflock);
  return true;

#else
  return false;

#endif
}

static inline fu32
validate_coords(const f_xyz *gps, size_t len, fu32 conf)
{
  if (len != sizeof(f_xyz))
  {
    return fc_mask(GPS_Malformed);
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
  if ((conf & option(Vigilant_Mode)) && !beyond(Landed) &&
      (gps->z > MAX_SEA || gps->z < MIN_SEA))
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
    tx_queue_send(&seds_syscall, &rep, TX_NO_WAIT);
    return SEDS_ERR;
  }

  fc_lock(&rfboard.rflock);
  rfboard.coords_buf = *(f_xyz *)data;
  rfboard.updated = true;
  fc_concede(&rfboard.rflock);

  sweetbench_start(10, 50);

#endif /* GPS_AVAILABLE */

  return SEDS_OK;
}

static inline void to_relative_coords(kf_gps *buf)
{
#ifdef GPS_AVAILABLE
  buf->lon = fabsf(buf->lon - rfboard.rail.lon);
  buf->lat = fabsf(buf->lat - rfboard.rail.lat);
#endif
}

static inline void
watch_for_gps_packets(fu32 conf, fu32 *acc, fu32 *ctr)
{
#ifdef GPS_AVAILABLE

  if ((conf & option(GPS_Available)) && fetch_gps_data(&meas.gps))
  {
    *acc += timer_exchange(GPSWatchdog);

    if (!within(rfboard.rail.lat - meas.gps.lat, GPS_RAIL_TOLER) ||
        !within(rfboard.rail.lon - meas.gps.lon, GPS_RAIL_TOLER))
    {
      const float rep[2] = {rfboard.rail.lat, rfboard.rail.lon};
      char buf[MAX_METRIC_MESSAGE_SIZE] = id "new GPS reference: ";

      seds_ftoa4(buf + strlen(buf) + 1, rep, 2);
      message(buf, true);
    }

    rfboard.rail = meas.gps;

    if (++*ctr % 61 == 0 /* Golang! */)
    {
      float avg = *(float *)acc / *(float *)ctr;
      log_metric(id "GPS interval", (fi32) avg, false);
    }
  }

#endif /* GPS_AVAILABLE */
}


/* General packet handling */

#if defined(TELEMETRY_ENABLED) || defined(FAKESTATION)

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
              ? tx_queue_send(&seds_syscall, &msg, TX_NO_WAIT)
              : INVALID_MESSAGE_STATUS;
  }

  tx_thread_priority_change(&telemetry_thread,
                            TLMT_PRIORITY, &tlmt_old_pr);

#else /* MESSAGE_BATCHING_ENABLED */

  msg = decode_cmd(data);

  st = msg != Invalid_Message
            ? tx_queue_send(&seds_syscall, &msg, TX_NO_WAIT)
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
    message(id "biases blocked mid-flight", true);
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

static inline SedsResult
implicit_postinit(const uint8_t *data, size_t len)
{
  if (len != sizeof(uint8_t))
  {
    return SEDS_ERR;
  }
  else if (*data == G_Armed)
  {
    fc_msg post = fc_mask(Postinit_Signal);
    tx_queue_send(&seds_syscall, &post, TX_NO_WAIT);
  }

  return SEDS_OK;
}

SedsResult on_fc_packet(const SedsPacketView *pkt, void *_)
{
  if (!pkt || !pkt->sender || !pkt->sender_len ||
      !pkt->payload || !pkt->payload_len)
  {
    return SEDS_HANDLER_ERROR;
  }

  // led_toggle(LED1_PORT, LED1_PIN);

  switch (pkt->ty)
  {
    case SEDS_DT_HEARTBEAT:
      return pulse_ground();
    case SEDS_DT_FLIGHT_STATE:
      return implicit_postinit(pkt->payload, pkt->payload_len);
    case SEDS_DT_GPS_DATA:
      return process_gps_packet(pkt->payload, pkt->payload_len);
    case SEDS_DT_FLIGHT_COMMAND:
      return dispatch_flight_cmd(pkt->payload, pkt->payload_len);
    case SEDS_DT_ASCENT_BIASES:
      return update_ascent_biases(pkt->payload, pkt->payload_len);
  }

  return SEDS_HANDLER_ERROR;
}

#endif /* TELEMETRY_ENABLED + FAKESTATION */


/* On-board sensor data validation */

static inline void
maybe_report_measm(fu32 conf, fc_msg code, devid kind)
{
  extern atomic_uint_fast16_t devctr_if[];

  if (!(conf & option(Using_Ascent_KF)) && kind == IMU)
  {
    return;
  }

  fu16 ctr = load(&devctr_if[kind], Rlx);

  if (code == fc_mask(Sensor_Measm_Code))
  {
    if ((conf & option(Reset_Failures)) && ctr > 0
        && fetch_sub(&devctr_if[kind], 1, Rel) == 0)
    {
      store(&devctr_if[kind], 0, Rel);
    }
  }
  else if (ctr <= TO_ABORT)
  {
    fetch_add(&devctr_if[kind], 1, Rel);

    if (conf & option(Measm_Reports))
    {
      tx_queue_send(&seds_syscall, &code, TX_NO_WAIT);
    }
  }
}

static inline bool
validate_gyro(const f_xyz *gyro, fu32 conf)
{
  fc_msg code = fc_mask(Sensor_Measm_Code);

  if (gyro->x > MAX_DPS || gyro->x < MIN_DPS)
  {
    code |= Bad_Attitude_X;
  }
  if (gyro->y > MAX_DPS || gyro->y < MIN_DPS)
  {
    code |= Bad_Attitude_Y;
  }
  if (gyro->z > MAX_DPS || gyro->z < MIN_DPS)
  {
    code |= Bad_Attitude_Z;
  }

  maybe_report_measm(conf, code, IMU);

  return code == fc_mask(Sensor_Measm_Code);
}

static inline bool
validate_accl(const f_xyz *accl, fu32 conf)
{
  fc_msg code = fc_mask(Sensor_Measm_Code);

  if (accl->x > MAX_ACC || accl->x < MIN_ACC)
  {
    code |= Bad_Accel_X;
  }
  if (accl->y > MAX_ACC || accl->y < MIN_ACC)
  {
    code |= Bad_Accel_Y;
  }
  if (accl->z > MAX_ACC || accl->z < MIN_ACC)
  {
    code |= Bad_Accel_Z;
  }

  maybe_report_measm(conf, code, IMU);

  return code == fc_mask(Sensor_Measm_Code);
}

static inline bool
validate_baro(const baro *baro, fu32 conf)
{
  fc_msg code = fc_mask(Sensor_Measm_Code);

  if (baro->prs > MAX_PRS || baro->prs < MIN_PRS)
  {
    code |= Bad_Pressure;
  }
  if (baro->alt > MAX_ALT || baro->alt < MIN_ALT)
  {
    code |= Bad_Altitude;
  }

  maybe_report_measm(conf, code, Baro);

  return code == fc_mask(Sensor_Measm_Code);
}

static inline conditional fu8
validate_all(const measm *buf, fu32 conf)
{
  return validate_baro(&buf->baro, conf) +
         validate_gyro(&buf->gyro, conf) +
         validate_accl(&buf->accl, conf);
}


/* Measurement logging */

static inline bool maybe_log_measm(devid dev, const void *buf)
{
  static const log_lookup mems[MEMS_Devices] = {
    [IMU]  = {  .tim_sd = IMULocal, 
                .tim_gnd = IMURemote,
                .size = 6,
                .kind_sd = SEDS_DT_IMU_LOCAL,
                .kind_gnd = SEDS_DT_IMU_DATA  },
    [Baro] = {  .tim_sd = BaroLocal,
                .tim_gnd = BaroRemote,
                .size = 3,
                .kind_sd = SEDS_DT_BAROMETER_LOCAL,
                .kind_gnd = SEDS_DT_BAROMETER_DATA  },
  };

  bool recorded = false;

#ifdef SD_AVAILABLE
  if (timer_probe(mems[dev].tim_sd, rates.sd))
  {
    sd_append_f32(mems[dev].kind_sd, buf, mems[dev].size);
    recorded = true;
  }
#endif

  if (timer_probe(mems[dev].tim_gnd, rates.gnd * 2)) /* Slow link */
  {
    log_f32(mems[dev].kind_gnd, mems[dev].size, buf);
    recorded = true;
  }

  return recorded;
}


/* Distribution for Ascent */

static inline void for_ascent_update(fu32 conf)
{
  baro baro_suspect;

  if (try_fetch_baro(&baro_suspect) && validate_baro(&baro_suspect, conf))
  {
    fc_lock(&meas_locks[Baro]);
    meas.baro = baro_suspect;
    fc_concede(&meas_locks[Baro]);
  }
  else return;

  tx_event_flags_set(&eval_stage, Baro_Mask, TX_OR);

  maybe_log_measm(Baro, &meas.baro);

  sweetbench_catch(8);

  if (conf & Eval_Focused)
  {
    tx_thread_relinquish();
  }
}

static inline void for_ascent_predict(fu32 conf, fu8 *imu)
{
  static f_xyz accum_gyro, accum_accl;

  f_xyz suspect_gyro, suspect_accl;

  sweetbench_start(8);

  if (try_fetch_gyro(&suspect_gyro) && validate_gyro(&suspect_gyro, conf))
  {
    accum_gyro = suspect_gyro;
    *imu |= Gyro_Mask;
  }

  if (try_fetch_accl(&suspect_accl) && validate_accl(&suspect_accl, conf))
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

  meas.accl = accum_accl;
  meas.gyro = accum_gyro;

  fc_concede(&meas_locks[IMU]);

  tx_event_flags_set(&eval_stage, Gyro_Mask | Accl_Mask, TX_OR);

  *imu &= ~(Gyro_Mask | Accl_Mask);

  maybe_log_measm(IMU, &meas.accl);

  if (conf & option(Eval_Focused))
  {
    tx_thread_relinquish();
  }
}


/* Distribution for Descent and itself */

static inline void descent_full_cycle(fu32 conf)
{
  fu32 stage = 0;

  sweetbench_start(9);

  if (try_fetch_baro(&meas.baro))
  {
    stage = EVALUATION_STAGED;
    maybe_log_measm(Baro, &meas.baro);
  }

  if ((conf & option(GPS_Available)) && fetch_gps_data(&meas.gps))
  {
    to_relative_coords(&meas.gps);

    if (!(conf & option(Vigilant_Mode)))
    {
      stage = EVALUATION_STAGED;
    }
  }

  if (stage == EVALUATION_STAGED)
  {
    float dt = fsec(timer_exchange(DescentKF));
    descent_predict(dt);
    descent_update();
    evaluate_rocket_state(conf, dt);
    sweetbench_catch(9);
  }
  else tx_thread_relinquish();
}


/* Stage 0 of fill sequence */

static inline void data_streaming_mode(void)
{
  fu32 acc_baro = 0.0f, acc_gps = 0.0f;
  fu32 ctr_baro = 0, ctr_gps = 0;
  fu32 conf = 0, imu = 0;

  timer_update(Auxiliary);

  MrAnalog ((conf & option(Postinit_Requested)) ||
            (conf & option(Launch_Requested)))
  {
    if (try_fetch_gyro(&meas.gyro))
    {
      imu |= Gyro_Mask;
      validate_gyro(&meas.gyro, conf);
    }
    if (try_fetch_accl(&meas.accl))
    {
      imu |= Accl_Mask;
      validate_accl(&meas.accl, conf);
    }
    if (try_fetch_baro(&meas.baro))
    {
      validate_baro(&meas.baro, conf);
      maybe_log_measm(Baro, &meas.baro);

      acc_baro += timer_exchange(Auxiliary);

      if (++ctr_baro % 61 == 0 /* Golang! */)
      {
        float avg = (float)acc_baro / (float)ctr_baro;
        log_metric(id "Baro interval", (fi32) avg, false);
      }
    }
    if ((imu & (Accl_Mask | Gyro_Mask)) && maybe_log_measm(IMU, &meas.accl))
    {
      imu = 0;
    }
    watch_for_gps_packets(conf, &acc_gps, &ctr_gps);

    tx_thread_relinquish();

    conf = load(&g_conf, Acq);
  }
}


/* Stage 1 of fill sequence */

static inline void post_initialization(void)
{
  f_xyz accl_acc = {0};
  fu32 accl_ctr = 0, gps_ctr = 0, gps_acc = 0;

  timer_update(Auxiliary);

  MrAnalog (timer_fetch(Auxiliary) > POSTINIT_DURATION)
  {
    fu32 conf = load(&g_conf, Acq);

    if (try_fetch_accl(&meas.accl))
    {
      maybe_log_measm(IMU, &meas.accl);

      if (validate_accl(&meas.accl, conf))
      {
        accl_acc.x += meas.accl.x;
        accl_acc.y += meas.accl.y;
        accl_acc.z += meas.accl.z;
        ++accl_ctr;
      }
    }
    watch_for_gps_packets(conf, &gps_acc, &gps_ctr);

    tx_thread_relinquish();
  }

  accl_acc.x /= accl_ctr;
  accl_acc.y /= accl_ctr;
  accl_acc.z /= accl_ctr;

  accel_to_quaternion(&accl_acc);

  if (fetch_add(&sm.flight, 1, Acq) != Armed - 1)
  {
    store(&sm.flight, Armed, Rlx);
    message(id "unusual startup sequence", true);
  }

  fetch_and(&g_conf, ~option(Postinit_Requested), Rel);
  message(id "armed, awaiting launch signal", true);
}


/* Task */

static inline bool fill_sequence_states(void)
{
  fu32 conf;
  fc_msg cmd = fc_mask(Reinit_Sensors);

  do
  {
    if (!beyond(Startup))
    {
      tx_queue_send(&seds_syscall, &cmd, TX_NO_WAIT);
    }

    data_streaming_mode();
    message(id "left streaming mode", true);

    conf = load(&g_conf, Acq);

    if (conf & option(Postinit_Requested))
    {
      tx_queue_send(&seds_syscall, &cmd, TX_NO_WAIT);
      post_initialization();
    }
  }
  MrAnalog (beyond(Startup) && (conf & option(Launch_Requested)));

  MrAnalog (request_ignition() == SEDS_OK)
    ;
  message(id "ignition requested, in flight mode", true);

  return false;
}

void distribution_entry(ULONG _)
{
  fu8 imu = 0;
  fu32 conf;

  fill_sequence_states();

  MrAnalog (WE_ARE_SO_BACK)
  {
    conf = load(&g_conf, Acq);

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
    log_err(id "stack %s %u", critical, st);
    return IT_IS_NOW_OVER;
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
    log_err(id "task %s %u", critical, st);
    return IT_IS_NOW_OVER;
  }

  return TX_SUCCESS;
}