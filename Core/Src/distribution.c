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

static f_xyz rail = {0};
static atomic_uint_fast16_t gps_mask = 0xFF00u;
static struct coords gps_ring[GPS_RING_SIZE] = {0};

#endif /* GPS_AVAILABLE */


#ifdef TELEMETRY_ENABLED
#ifdef TELEMETRY_CMD_COMPAT

enum remote_cmd_compat : uint8_t
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
};

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

static inline enum message decode_cmd(const uint8_t *raw)
{
  return (enum message)U32(raw[0], raw[1], raw[2], raw[3]);
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
  memcpy(&gps_ring[i], buf, sizeof(struct coords));

  i = (i + 1) & GPS_RING_SIZE_MASK;
  fu8 cons = load(&gps_mask, Acq) >> 8;

  if (i != cons)
  {
    fetch_add(&gps_mask, 1, Rel);
    idx = i;
  }
}

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

  i = (i + n) & GPS_RING_SIZE_MASK;
  idx = i;

  *buf = gps_ring[i];

  fetch_or(&gps_mask, CLEAR_IDX, Rlx);
  return n;
}

/*
 * GPS sanity check against NEO-M9N data range.
 */
static inline fc_msg
validate_gps_absolute(const struct coords *gps)
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
validate_gps_relative(const struct coords *gps)
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

  if (len != sizeof(struct coords))
  {
    rep |= GPS_Malformed;
  }
  else
  {
    fu32 conf = load(&g_conf, Acq);

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

  fetch_or(&g_conf, option(GPS_Available), Rlx);

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
SedsResult on_fc_packet(const SedsPacketView *pkt, void *user)
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

  else
  {
    return SEDS_HANDLER_ERROR;
  }
}

#endif /* TELEMETRY_ENABLED */


/*
 * Gyroscope sanity check against its data range.
 */
static inline fc_msg
validate_gyro(const struct coords *gyro, fu32 conf)
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
validate_accl(const struct coords *accl, fu32 conf)
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
         validate_accl(&buf->mode.accl, conf);
}


/*
 * Distribution loop that executes before the Flight
 * Computer receives command to launch. After exiting
 * the loop, this function performs final sanity checks
 * and sends ignition signal to the Valve board.
 */
static inline void pre_launch(void)
{
  fu32 ctr = 0, conf = 0;

  float acc_baro = 0, acc_gps = 0;
  fu32 ctr_baro = 0, ctr_gps = 0;

  task_loop (conf & option(Launch_Triggered))
  {
    fu32 st = fc_mask(Sensor_Measm_Code);

    if (fetch_gyro(&meas.gyro))
    {
      st |= validate_gyro(&meas.gyro, conf);
      log_measm(SEDS_DT_GYRO_DATA, &meas.gyro);
    }

    if (fetch_accl(&meas.mode.accl))
    {
      st |= validate_accl(&meas.mode.accl, conf);
      log_measm(SEDS_DT_ACCEL_DATA, &meas.mode.accl);
    }

    if (fetch_baro(&meas.baro))
    {
      acc_baro += fsec(timer_exchange(IntervalBaro));
      ++ctr_baro;

      st |= validate_baro(&meas.baro, conf);
      log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);
    }

    if (st != fc_mask(Sensor_Measm_Code))
    {
      log_err(pilot "malformed measm: %u", fc_unmask(st));
    }

#ifdef GPS_AVAILABLE

    if (conf & option(GPS_Available) &&
        fetch_gps_data(&meas.mode.gps))
    {
      acc_gps += fsec(timer_exchange(IntervalGPS));

      rail = meas.mode.gps;

      if (!within(rail.x - meas.mode.gps.x, GPS_RAIL_TOLER) ||
          !within(rail.y - meas.mode.gps.y, GPS_RAIL_TOLER))
      {
        log_err(id "new GPS reference "
                   "LAT: %f, LON: %f", rail.x, rail.y);
      }
    }

#endif /* GPS_AVAILABLE */

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

  task_loop(request_ignition() == SEDS_OK)
    ;

  log_msg(id "ignition requested, in flight mode");
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
  struct coords suspect;

  sweetbench_start(8);

  if (fetch_gyro(&suspect))
  {
    st = validate_gyro(&suspect, conf);

    if (st == fc_mask(Sensor_Measm_Code))
    {
      meas.gyro = suspect;
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
      meas.mode.accl = suspect;
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
      ascent_cycle_update(conf, imu);
    }
    else
    {
      tx_thread_relinquish();
    }
    return;
  }

  *imu &= ~IMU_ID;

  sm.dt = fsec(timer_exchange(AscentKF));

  ascent_predict(sm.dt);
  *imu |= ASCENT_PREDICT_DONE;

  log_measm(SEDS_DT_GYRO_DATA, &meas.gyro);
  log_measm(SEDS_DT_ACCEL_DATA, &meas.mode.accl);

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
    else
    {
      tx_queue_send(&shared, &st, TX_NO_WAIT);
    }

    log_measm(SEDS_DT_BAROMETER_DATA, &meas.baro);
  }

#ifdef GPS_AVAILABLE

  if ((conf & option(GPS_Available)) &&
      fetch_gps_data(&meas.mode.gps))
  {
    distance_from_rail(&meas.mode.gps);
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
  else
  {
    tx_thread_relinquish();
  }
}


/*
 * Operates flight states before launch, runs KF
 * data distribution loops.
 */
void distribution_entry(ULONG input)
{
  (void)input;

  log_msg(id "started");

  fu32 conf = load(&g_conf, Acq);

  fu8 imu = 0;

  if (!(conf & option(Launch_Triggered)))
  {
    pre_launch();
  }

  task_loop(DO_NOT_EXIT)
  {
    conf = load(&g_conf, Acq);

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