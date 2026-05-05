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


TX_THREAD evaluation_task;
TX_EVENT_FLAGS_GROUP eval_stage;

void cm_align *kfpool_buf = NULL;

kf_svec sv[STATE_HISTORY] = {0};
sv_meta sm = {Startup, G_Startup, 0, 0, 0};

uncached volatile float kalt, kvel;


/* FSM helpers */

static inline void handle_spurious(fu32 mode, const char *cond)
{
  if (sm.confidence >= SPURIOUS_THRESHOLD)
  {
    ++sm.spilled_milk;

    char buf[MAX_SPURIOUS_REPORT_SIZE];
    snprintf(buf, sizeof buf, id "fluc: %u, failed %s",
                                      current(), cond);
    message(buf, false);
  }

  if (mode & option(Consecutive_Samples))
  {
    sm.confidence = 0;
  }
  else if (sm.confidence > 0)
  {
    sm.confidence -= mind(sm.confidence, SPURIOUS_PENALTY);
  }
}

static inline void flight_advance(state promotion)
{
  sm.confidence = 0;

  if (fetch_add(&sm.flight, 1, Acq) != promotion - 1)
  {
    store(&sm.flight, promotion, Rlx);
    message(id "vigilant mode transition", false);
  }

  log_flight_state(to_global_state(promotion));
}


/* State transitions */

static inline void detect_boost(fu32 mode)
{
  Require(svec(0).vel >= LAUNCH_MIN_VEL);
  Require(svec(0).vel > svec(2).vel);
  Require(svec(2).vel > svec(4).vel);

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_LAUNCH)
  {
    flight_advance(Launch);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}

static inline void detect_ascent(fu32 mode)
{
  Require(svec(0).vel >= ASCENT_MIN_VEL);
  Require(svec(0).vel > svec(3).vel);
  Require(svec(0).alt > svec(2).alt);
  Require(svec(2).alt > svec(4).alt);

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_ASCENT)
  {
    flight_advance(Ascent);
  }
}

static inline void detect_coast(fu32 mode)
{
  Require(svec(0).alt > svec(2).alt);
  Require(svec(0).vel > COAST_MIN_VEL);
  Require(svec(0).vel < svec(5).vel);

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_COAST)
  {
    flight_advance(Coast);
  }
}

static inline void detect_apogee(fu32 mode)
{
  Require(within(svec(2).vel, APOGEE_MAX_VEL));
  Require(within(svec(0).vel, APOGEE_MAX_VEL));

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_APOGEE)
  {
    descent_initialize(mode);
    flight_advance(Apogee);
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}

static inline void detect_descent(fu32 mode)
{
  Require(svec(0).vel <= -DESCENT_MIN_VEL);
  Require(svec(0).alt < svec(2).alt);
  Require(svec(2).alt < svec(4).alt);

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_DESCENT)
  {
    release_parachute(false);
    flight_advance(Descent);
  }
}

static inline void detect_reefing(fu32 mode)
{
  Require(svec(0).alt <= REEF_TARGET_ALT);
  Require(svec(0).alt < svec(2).alt);
  Require(svec(2).alt < svec(4).alt);

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_REEF)
  {
    expand_parachute(false);
    flight_advance(Reefing);
  }
}

static inline void detect_landed(fu32 mode)
{
  float dh_1 = svec(0).alt - svec(1).alt;
  float dv_1 = svec(0).vel - svec(1).vel;
  float dh_4 = svec(2).alt - svec(6).alt;
  float dv_4 = svec(2).vel - svec(6).vel;

  Require(within(dh_1, ALT_TOLER));
  Require(within(dh_4, ALT_TOLER));
  Require(within(dv_1, VEL_TOLER));
  Require(within(dv_4, VEL_TOLER));

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_LANDED)
  {
    flight_advance(Landed);
    tx_thread_sleep(RECOVERY_ANNOUNCE_DELAY);
  }
}

static inline void announce_recovery(fu32 mode)
{
  flight_advance(Recovery);
  message(id "announcing recovery", true);
  fetch_or(&g_conf, option(SD_Pipeline_Reset), Rel);
  tx_thread_sleep(RECOVERY_ANNOUNCE_DELAY);
}

static inline void report_lowpass_gps(fu32 mode)
{
#ifdef GPS_AVAILABLE

  if (mode & option(GPS_Available))
  {
    f_xyz lowpass = {
      svec(0).gps.lat, svec(0).gps.lon, meas.gps.sea
    };

    log_f32(SEDS_DT_GPS_DATA, 3, &lowpass);
  }

  tx_thread_sleep(RECOVERY_ANNOUNCE_DELAY / 100);

#endif
}


/* Vigilant mode */

static inline bool maybe_force(fu32 mode, float alt)
{
  return !beyond(Armed) && alt >= FLYING_ALTITUDE
                        && !within(alt - svec(3).alt, ALT_TOLER);
}

static inline void vigilant_watchdog(fu32 mode, state now)
{
  float alt = svec(0).alt;
  float vel = svec(0).vel;

  if (sm.spilled_milk > ALEX_THRESHOLD || alt > VIGILANT_MAX_ALT
                                       || alt < VIGILANT_MIN_ALT)
  {
    alt = meas.baro.alt;
  }

  if (now < Descent && vel < -VIGILANT_MIN_VEL && alt < svec(7).alt)
  {
    release_parachute(maybe_force(mode, alt));
    descent_initialize(mode);
    flight_advance(Descent);
  }
  else if (now < Reefing && alt <= REEF_TARGET_ALT
                         && (vel < 0.0f || alt < svec(4).alt))
  {
    if (now < Descent)
    {
      release_parachute(maybe_force(mode, alt));
      descent_initialize(mode);
      flight_advance(Descent);
      tx_thread_sleep(URGENT_DEPLOYMENT_DELAY);
    }

    if (expand_parachute(false))
    {
      flight_advance(Reefing);
    }
  }
}


/* FSM */

static inline void propel_kalman_state(fu32 conf)
{
  conditional fu16 kind_sd, kind_gnd, elements;
  float tmp[MAX_STATE];

  if (conf & option(Using_Ascent_KF))
  {
    *((quat *)tmp) = qv;
    tmp[4] = svec(0).alt;
    tmp[5] = svec(0).vel;
    kind_sd = SEDS_DT_ASCENT_LOCAL;
    kind_gnd = SEDS_DT_ASCENT_STATE;
    elements = EKF_STATE;
  }
  else
  {
    tmp[0] = svec(0).gps.lon;
    tmp[1] = svec(0).gps.lat;
    tmp[2] = svec(0).alt;
    tmp[3] = svec(0).vel;
    kind_sd = SEDS_DT_DESCENT_LOCAL;
    kind_gnd = SEDS_DT_DESCENT_STATE;
    elements = DKF_STATE;
  }

#ifdef SD_AVAILABLE
  if (timer_probe(KFLocal, rates.sd))
  {
    sd_append_f32(kind_sd, tmp, elements);
  }
#endif

  if (timer_probe(KFRemote, rates.gnd))
  {
    log_f32(kind_gnd, elements, tmp);
  }

  sm.idx = (sm.idx + 1) & STATE_HISTORY_MASK;
}



void evaluate_rocket_state(fu32 conf)
{
  kalt = svec(0).alt;
  kvel = svec(0).vel;

  state curr = current();

  if (curr < Reefing && (conf & option(Monitor_Altitude)))
  {
    vigilant_watchdog(conf, curr);
  }

  switch (curr)
  {
    case Armed:     detect_boost(conf);         break;
    case Launch:    detect_ascent(conf);        break;
    case Ascent:    detect_coast(conf);         break;
    case Coast:     detect_apogee(conf);        break;
    case Apogee:    detect_descent(conf);       break;
    case Descent:   detect_reefing(conf);       break;
    case Reefing:   detect_landed(conf);        break;
    case Landed:    announce_recovery(conf);    break;
    case Recovery:  report_lowpass_gps(conf);   break;
    default:
      log_err(id "state %u cannot be evaluated", curr);
      return;
  }

  propel_kalman_state(conf);
}

static inline void enter_flight_mode(fu32 conf)
{
  if (conf & option(Launch_Requested))
  {
    sm.idx = (sm.idx - 1) & STATE_HISTORY_MASK;
    message(id "re-entered flight mode", true);
  }
  else
  {
    ascent_initialize(conf);
    message(id "received launch signal", true);
    fetch_or(&g_conf, option(Launch_Requested), Rel);

#ifdef DESCENT_TEST
    descent_initialize(conf);
    store(&sm.flight, Apogee, Rel);
    fetch_and(&g_conf, ~option(Using_Ascent_KF), Rel);
#endif
  }
}


/* Task */

void evaluation_entry(ULONG _)
{
  fu8 accum = 0;
  fu32 conf = load(&g_conf, Acq);

  enter_flight_mode(conf);

  MrAnalog (conf & option(Eval_Abort_Flag))
  {
    ULONG done, request = accum & EVALUATION_STAGED
                                ? Baro_Mask
                                : Gyro_Mask | Accl_Mask;

    if (tx_event_flags_get(&eval_stage, request,
                           TX_OR_CLEAR, &done,
                           TX_WAIT_FOREVER) != TX_SUCCESS)
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
    else if ((accum & (Gyro_Mask | Accl_Mask)) == (Gyro_Mask | Accl_Mask))
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

  const char *critical = "creation failure:";

  st = tx_byte_allocate(byte_pool, (VOID **)&pointer,
                        EVAL_STACK_BYTES, TX_NO_WAIT);

  if (st != TX_SUCCESS)
  {
    log_die(id "stack %s %u", critical, st);
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