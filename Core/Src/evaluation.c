/* Core/Src/evaluation.c */

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
sv_meta sm = {Suspended, G_Startup, 1, 0, 0};


/* FSM helpers */

static inline void detect_spurious(fu32 mode)
{
  if (sm.samp <= SPURIOUS_THRESHOLD || sm.ev_step <= 0)
  {
    return;
  }

  char buf[MAX_SPURIOUS_REPORT_SIZE];

  sprintf(buf, id "confirm broke at %d, state %u",
                           sm.ev_step, current());

  log_msg(buf);

  if (mode & option(Consecutive_Samples))
  {
    sm.samp = sm.ev_step = 0;
  }
}

/* Step through in state evaluation. Reduces branching.
 */
static inline forceinline bool forward(void)
{
  return ++sm.ev_step > 0;
}

static inline void flight_advance(state promotion)
{
  sm.samp = sm.ev_step = 0;

  if (fetch_add(&sm.flight, 1, Acq) != promotion - 1)
  {
    store(&sm.flight, promotion, Rlx);
    log_critical(id "vigilant mode deployment:");
  }

  log_flight_state(to_global_state(promotion));
}


/* Evaluation routines */

static inline void vigilant_evaluate_altitude(fu32 mode)
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

    if (mode & option(Parachute_Deployed)
        && expand_parachute())
    {
      flight_advance(Reefing);
    }
    else if (release_parachute())
    {
      flight_advance(Descent);

      descent_initialize();
      tx_thread_sleep(URGENT_DEPLOYMENT_DELAY);

      flight_advance(Reefing);      
      expand_parachute();
    }
  }
  else if (!(mode & option(Confirm_Altitude)))
  {
    fetch_or(&g_conf, option(Confirm_Altitude), Rlx);
  }
  else if (!(mode & option(Parachute_Deployed))
           && release_parachute())
  {
    flight_advance(Descent);
    descent_initialize();
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


/* State transitions */

static inline void detect_launch(void)
{
  if (svec(0).vel >= LAUNCH_MIN_VEL     then
      svec(0).vel > svec(1).vel         then
      svec(1).vel > svec(2).vel)
  {
    flight_advance(Launch);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}

static inline void detect_ascent(fu32 mode)
{
  if (svec(0).vel > svec(1).vel         then
      svec(1).vel > svec(2).vel         then
      svec(0).alt > svec(1).alt         then
      svec(1).alt > svec(2).alt         then
      ++sm.samp >= MIN_SAMP_ASCENT)
  {
    flight_advance(Ascent);
  }
  else detect_spurious(mode);
}

static inline void detect_burnout(fu32 mode)
{
  if (svec(0).vel >= BURNOUT_MIN_VEL    then
      svec(0).alt > svec(1).alt         then
      svec(0).vel < svec(1).vel         then
      svec(1).vel < svec(2).vel         then
      ++sm.samp >= MIN_SAMP_BURNOUT)
  {
    flight_advance(Burnout);
  }
  else detect_spurious(mode);
}

static inline void detect_apogee(void)
{
  const fu8 eval_depth = 4;
  float absvel[eval_depth];

  for (fu8 k = 0; k < eval_depth; ++k)
  {
    absvel[k] = fabsf(svec(k).vel);
  }

  if (absvel[0] <= APOGEE_MAX_VEL       then
      absvel[0] < absvel[1]             then
      absvel[1] < absvel[2]             then
      absvel[2] < absvel[3])
  {
    flight_advance(Apogee);
    descent_initialize();
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}

static inline void detect_descent(fu32 mode)
{
  const fu8 eval_depth = 3;
  float absvel[eval_depth];

  for (fu8 k = 0; k < eval_depth; ++k)
  {
    absvel[k] = fabsf(svec(k).vel);
  }

  if (svec(0).alt < svec(1).alt         then
      svec(1).alt < svec(2).alt         then
      absvel[0] > absvel[1]             then
      absvel[1] > absvel[2]             then
      ++sm.samp >= MIN_SAMP_DESCENT)
  {
    release_parachute();
    flight_advance(Descent);
  }
  else detect_spurious(mode);
}

static inline void detect_reef(fu32 mode)
{
  if (svec(0).alt <= REEF_TARGET_ALT    then
      svec(0).alt < svec(1).alt         then
      ++sm.samp >= MIN_SAMP_REEF)
  {
    expand_parachute();
    flight_advance(Reefing);
  }
  else detect_spurious(mode);
}

static inline void detect_landed(fu32 mode)
{
  float dh = svec(0).alt - svec(1).alt;
  float dv = svec(0).vel - svec(1).vel;

  if (fabsf(dh) <= ALT_TOLER            then
      fabsf(dv) <= VEL_TOLER            then
      ++sm.samp >= MIN_SAMP_LANDED)
  {
    flight_advance(Landed);
    sm.ev_step = -STABILIZATION_PAD;
  }
  else detect_spurious(mode);
}

static inline void stabilize(fu32 steps)
{
  float dh = svec(0).alt - svec(1).alt;
  float dv = svec(0).vel - svec(1).vel;

  if (fabsf(dh) <= ALT_TOLER            then
      fabsf(dv) <= VEL_TOLER            then
      ++sm.samp >= steps)
  {
    flight_advance(Recovery);
  }
}


/* FSM */

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

void evaluate_rocket_state(fu32 conf)
{
  state curr = current();

  if (curr < Reefing && (conf & option(Monitor_Altitude)))
  {
    vigilant_evaluate_altitude(conf);
  }

  switch (curr)
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
    stabilize(STABILIZATION_STEPS);
    break;
  case Recovery:
    crew_send_coords(conf);
    break;
  default: break;
  }

  propel_kalman_state(conf);
}

static inline void enter_flight_mode(fu32 conf)
{
  if (conf & option(Launch_Requested))
  {
    sm.idx = (sm.idx - 1) & STATE_HISTORY_MASK;
  }
  else
  {
    ascent_initialize(conf);
    log_critical(id "received launch signal");

    if (fetch_add(&sm.flight, 1, Acq) != Awaiting - 1)
    {
      store(&sm.flight, Awaiting, Rlx);
      log_err(id "unusual startup sequence");
    }
    
    fetch_or(&g_conf, option(Launch_Requested), Rel);
  }
}


/* Task */

void evaluation_entry(ULONG st)
{
  fu8 accum = 0;
  fu32 conf = load(&g_conf, Acq);

  enter_flight_mode(conf);

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
      accum &= ~(EVALUATION_STAGED | Baro_Mask);

      ascent_update();

      conf = fetch_and(&g_conf,
                       ~option(Ascent_KF_Staged), AcqRel);

      sweetbench_catch(3);
      evaluate_rocket_state(conf);
      sweetbench_start(3, 50, true);
    }
    else if (accum & (Gyro_Mask | Accl_Mask))
    {
      accum |= EVALUATION_STAGED;
      accum &= ~(Gyro_Mask | Accl_Mask);

      conf = fetch_or(&g_conf,
                      option(Ascent_KF_Staged), AcqRel);

      ascent_predict(fsec(timer_exchange(AscentKF)), conf);
    }
  }
}

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