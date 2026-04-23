/*
 * Evaluation Task
 */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"

#define id "EV "
#define id_vigilant "VM "


TX_THREAD evaluation_task;
TX_EVENT_FLAGS_GROUP eval_stage;

void tx_align *kfpool_buf = NULL;

kf_svec sv[STATE_HISTORY] = {};
sv_meta sm = {1, 0, Suspended};


/*
 * Vigilant mode routine. Can perform urgent deployments
 * and skip states.
 */
static inline void evaluate_altitude(fu32 mode)
{
  float last = svec(0).alt;

  if (last > VIGILANT_MAX_ALT || last < VIGILANT_MIN_ALT)
  {
    last = meas.baro.alt;
  }

  if (!beyond(Launch) || last > svec(1).alt
                      || svec(1).alt > svec(2).alt)
  {
    if (mode & option(Consecutive_Samples) &&
        mode & option(Confirm_Altitude))
    {
      fetch_and(&g_conf, ~option(Confirm_Altitude), Rlx);
    }

    return;
  }

  if (last <= REEF_TARGET_ALT &&
      !(mode & option(Parachute_Expanded)))
  {
    /* We fell below reefing altitude. Depeding on
     * how much we missed, peform the deployments. */

    if (mode & option(Parachute_Deployed))
    {
      expand_parachute();

      sm.flight = Reefing;
      log_flight_state(sm.flight);
    }
    else
    {
      release_parachute();

      sm.flight = Descent;
      log_flight_state(sm.flight);

      descent_initialize();

      tx_thread_sleep(URGENT_DEPLOYMENT_DELAY);
      expand_parachute();

      sm.flight = Reefing;
      log_flight_state(sm.flight);
    }
  }
  else if (!(mode & option(Confirm_Altitude)))
  {
    fetch_or(&g_conf, option(Confirm_Altitude), Rlx);
  }
  else if (!(mode & option(Parachute_Deployed)))
  {
    release_parachute();
    
    sm.flight = Descent;
    log_flight_state(sm.flight);
    
    descent_initialize();
  }
}

/*
 * Monitors if minimum thresholds for velocity and
 * acceleration were exceded.
 */
static inline void detect_launch(void)
{
  if (svec(0).vel >= LAUNCH_MIN_VEL &&
      meas.accl.z >= LAUNCH_MIN_VAX)
  {
    sm.flight = Launch;
    log_flight_state(sm.flight);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}

/*
 * Monitors height and velocity increase consistency.
 */
static inline void detect_ascent(fu32 mode)
{
  if (svec(0).vel > svec(1).vel &&
      svec(1).vel > svec(2).vel &&
      svec(0).alt > svec(1).alt &&
      svec(1).alt > svec(2).alt)
  {
    if (++sm.samp >= MIN_SAMP_ASCENT)
    {
      sm.flight = Ascent;
      sm.samp = 0;
      log_flight_state(sm.flight);
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Launch;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Monitors if minimum threshold for velocity and
 * maximum threshold for acceleration were passed.
 * Checks for height increase and velocity decrease
 * consistency.
 */
static inline void detect_burnout(fu32 mode)
{
  if (svec(0).vel >= BURNOUT_MIN_VEL &&
      meas.accl.z <= BURNOUT_MAX_VAX &&
      svec(0).alt > svec(1).alt &&
      svec(0).vel < svec(1).vel)
  {
    if (++sm.samp >= MIN_SAMP_BURNOUT)
    {
      sm.flight = Burnout;
      sm.samp = 0;
      log_flight_state(sm.flight);
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Burnout;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Initially monitors for continuing burnout and
 * for velocity to pass the minimum threshold.
 */
static inline void detect_apogee(void)
{
  if (svec(0).vel <= APOGEE_MAX_VEL &&
      svec(0).vel < svec(1).vel &&
      svec(1).vel < svec(2).vel &&
      svec(2).vel < svec(3).vel)
  {
    sm.flight = Apogee;
    log_flight_state(sm.flight);
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}

/*
 * Monitors for decreasing altitude and increasing velocity.
 */
static inline void detect_descent(fu32 mode)
{
  if (svec(0).alt < svec(1).alt &&
      svec(1).alt < svec(2).alt &&
      svec(0).vel > svec(1).vel &&
      svec(1).vel > svec(2).vel)
  {
    if (++sm.samp >= MIN_SAMP_DESCENT)
    {
      sm.flight = Descent;
      sm.samp = 0;
      release_parachute();
      log_flight_state(sm.flight);

      descent_initialize();
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Descent;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Monitors for falling below a specific altitude,
 * and checks for altitude consistency.
 */
static inline void detect_reef(fu32 mode)
{
  if (svec(0).alt <= REEF_TARGET_ALT &&
      svec(0).alt < svec(1).alt)
  {
    if (++sm.samp >= MIN_SAMP_REEF)
    {
      sm.flight = Reefing;
      sm.samp = 0;
      expand_parachute();
      log_flight_state(sm.flight);
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Reefing;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Monitors that all statistical metrics do not
 * deviate beyond allowed tolerance thresholds.
 */
static inline void detect_landed(fu32 mode)
{
  float dh = svec(0).alt - svec(1).alt;
  float dv = svec(0).vel - svec(1).vel;

  if (fabsf(dh) <= ALT_TOLER && fabsf(dv) <= VEL_TOLER)
  {
    if (++sm.samp >= MIN_SAMP_LANDED)
    {
      sm.flight = Landed;
      log_flight_state(sm.flight);

      fc_msg cmd = fc_mask(Evaluation_Focus);
      tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Landed;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


static inline void crew_send_coords(fu32 mode)
{
#ifdef GPS_AVAILABLE
  if (!(mode & option(GPS_Available)))
  {
    return;
  }

  log_measm(SEDS_DT_GPS_DATA, &meas.gps);

  tx_thread_sleep(LANDED_GPS_INTERVAL);

#else
  return;

#endif /* GPS_AVAILABLE */
}

/*
 * Logs and advances state in the ring buffer.
 */
static inline void propel_kalman_state(fu32 conf)
{
  if (conf & option(Using_Ascent_KF))
  {
    float tmp[EKF_STATE];

    *((quat *)tmp) = qv;
    tmp[EKF_STATE - 2] = svec(0).alt;
    tmp[EKF_STATE - 1] = svec(0).vel;

    log_ascent_state(tmp);
  }
  else log_descent_state(dkf_view(&svec(0)));

  sm.idx = (sm.idx + 1) & STATE_HISTORY_MASK;
}

/*
 * Finite-state machine for state transition.
 * Before leaving, logs state vector just used.
 */
void evaluate_rocket_state(fu32 conf)
{
  if (conf & option(Monitor_Altitude))
  {
    evaluate_altitude(conf);
  }

  switch (sm.flight)
  {
  case Awaiting:
    detect_launch();
    break;
  case Launch:
    detect_ascent(conf);
    break;
  case Ascent:
    detect_burnout(conf);
    break;
  case Burnout:
    detect_apogee();
    break;
  case Apogee:
    detect_descent(conf);
    break;
  case Descent:
    detect_reef(conf);
    break;
  case Reefing:
    detect_landed(conf);
    break;
  case Landed:
    crew_send_coords(conf);
    break;
  default: break;
  }

  propel_kalman_state(conf);
}

/*
 * Initializes Ascent filter and signals Distribution
 * task to start performing data logistics. Idempotent.
 * If there is a re-entrancy, discards dirty KF buffer.
 */
static inline void enter_flight_state(fu32 conf)
{
  if (conf & option(Launch_Requested))
  {
    sm.idx = (sm.idx - 1) & STATE_HISTORY_MASK;
  }
  else
  {
    ascent_initialize();
    log_critical(id "received launch signal");

    if (satur_incr(sm.flight, Landed) != Awaiting)
    {
      sm.flight = Awaiting;
      log_err(id "unusual startup sequence");
    }
    
    fetch_or(&g_conf, option(Launch_Requested), Rel);
  }
}

/*
 * Suspends on a message queue and performs
 * data evaluation in accordance with global config.
 */
void evaluation_entry(ULONG input)
{
  (void)input;

  UINT st;
  fu8 accum = 0;
  fu32 conf = load(&g_conf, Acq);

  enter_flight_state(conf);

  task_loop (conf & option(Eval_Abort_Flag))
  {
    ULONG done, request = accum & EVALUATION_STAGED
                                ? Baro_Mask
                                : Gyro_Mask | Accl_Mask;

    if ((st = tx_event_flags_get(&eval_stage, request,
                                 TX_OR_CLEAR, &done,
                                 TX_WAIT_FOREVER)) != TX_SUCCESS)
    { continue; }

    accum |= done;

    if ((accum & EVALUATION_STAGED) && (accum & Baro_Mask))
    {
      ascent_update();

      accum &= ~(EVALUATION_STAGED | Baro_Mask);
      conf = fetch_and(&g_conf,
                       ~option(Ascent_KF_Staged), AcqRel);

      sweetbench_catch(3);
      evaluate_rocket_state(conf);
      sweetbench_start(3, 50, true);
    }
    else if (accum & (Gyro_Mask | Accl_Mask))
    {
      conf = fetch_or(&g_conf,
                      option(Ascent_KF_Staged), AcqRel);
      
      ascent_predict(fsec(timer_exchange(AscentKF)), conf);

      accum |= EVALUATION_STAGED;
      accum &= ~(Gyro_Mask | Accl_Mask);
    }
  }
}

/*
 * Creates a configurably-preemptive, cooperative Evaluation task
 * with defined parameters. Started by the Recovery task.
 */
UINT create_evaluation_task(TX_BYTE_POOL *byte_pool)
{
  UINT st;
  CHAR *pointer;

  if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                       EVAL_STACK_BYTES, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  st = tx_thread_create(&evaluation_task,
                        "Evaluation Task",
                        evaluation_entry,
                        EVAL_INPUT,
                        pointer,
                        EVAL_STACK_BYTES,
                        EVAL_PRIORITY,
                        EVAL_PREEMPT_THRESHOLD,
                        EVAL_TIME_SLICE,
                        TX_DONT_START);

  const char *critical = "creation failure:";

  if (st != TX_SUCCESS)
  {
    log_die(id "task %s %u", critical, st);
  }

  st = tx_event_flags_create(&eval_stage, id "E");

  if (st != TX_SUCCESS)
  {
    log_die(id "evflags %s %u", critical, st);
  }

  kfpool_buf = _sbrk(KF_POOL_SIZE);

  if (kfpool_buf == (void *)-1)
  {
    log_die(id "poolbuf %s %u", critical, st);
  }

#ifdef PARALLEL_PREDICT_UPDATE

  st = tx_byte_pool_create(&kfpool, id "P", kfpool_buf, KF_POOL_SIZE);

  if (st != TX_SUCCESS)
  {
    log_die(id "pool %s %u", critical, st);
  }

#endif /* PARALLEL_PREDICT_UPDATE */

  return TX_SUCCESS;
}