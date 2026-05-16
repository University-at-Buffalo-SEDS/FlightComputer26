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

static stats extremes = {0};

#ifndef NDEBUG
uncached volatile float kalt, kvel;
#endif


/* FSM helpers */

static inline void handle_spurious(fu32 mode, const char *cond)
{
  if (sm.confidence >= SPURIOUS_THRESHOLD)
  {
    ++sm.kf_deviations;

    char buf[MAX_SPURIOUS_REPORT_SIZE];
    snprintf(buf, sizeof buf, id "fluc: %u, failed %s",
                                      current(), cond);
    message(buf, false);
  }

  if (mode & option(Eval_Successive))
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

  if (fetch_add(&sm.flight, 1, AcqRel) != promotion - 1)
  {
    store(&sm.flight, promotion, Rel);
    message(id "vigilant mode transition", false);
  }

  log_metric(id "new flight state", promotion, true);
  log_flight_state(to_global_state(promotion));
}


/* State transitions */

static inline void detect_boost(fu32 mode)
{
  if (mode & option(Velocity_Checks))
  {
    Require(fabsf(svec(0).vel) >= LAUNCH_MIN_VEL);
    Require(fabsf(svec(0).vel) > fabsf(svec(2).vel));
    Require(fabsf(svec(2).vel) > fabsf(svec(4).vel));
  }
  else
  {
    Require(svec(0).alt >= LAUNCH_MIN_ALT);
    Require(svec(0).alt > svec(2).alt);
    Require(svec(2).alt > svec(4).alt);
  }

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_LAUNCH)
  {
    flight_advance(Launch);
    timer_update(Auxiliary);
    extremes.max_vel = svec(0).vel;
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}

static inline void detect_ascent(fu32 mode)
{
  if (mode & option(Velocity_Checks))
  {
    Require(fabsf(svec(0).vel) >= ASCENT_MIN_VEL);
    Require(fabsf(svec(0).vel) > fabsf(svec(3).vel));
  }
  else
  {
    float dh_0_3 = svec(0).alt - svec(3).alt;
    float dh_4_7 = svec(4).alt - svec(7).alt;
    Require(dh_0_3 > dh_4_7);
  }

  Require(svec(0).alt > svec(3).alt);
  Require(svec(3).alt > svec(6).alt);

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_ASCENT)
  {
    flight_advance(Ascent);
    extremes.max_vel = maxd(svec(0).vel, extremes.max_vel);
  }
}

static inline void detect_coast(fu32 mode)
{
  extremes.max_vel = maxd(svec(0).vel, extremes.max_vel);

  if (mode & option(Velocity_Checks))
  {
    Require(svec(0).vel < svec(6).vel);
    sm.confidence += 1;
  }

  if (timer_fetch(Auxiliary) >= MOTOR_BURN_TIME &&
      (!(mode & option(Velocity_Checks)) || sm.confidence >= MIN_SAMP_COAST))
  {
    flight_advance(Coast);
    log_metric(id "maxvel guess", (fi32) extremes.max_vel, false);
  }
}

static inline void detect_apogee(fu32 mode)
{
  if (mode & option(Velocity_Checks))
  {
    Require(within(svec(2).vel, APOGEE_MAX_VEL));
    Require(within(svec(0).vel, APOGEE_MAX_VEL));
  }
  else
  {
    Require(svec(0).alt > VIGILANT_MIN_APG);

    float dh_0_2 = svec(0).alt - svec(2).alt;
    float dh_2_4 = svec(2).alt - svec(4).alt;

    Require(within(dh_0_2, APOGEE_ALT_TLR));
    Require(within(dh_2_4, APOGEE_ALT_TLR));
  }

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_APOGEE)
  {
    descent_initialize(mode);
    flight_advance(Apogee);
    extremes.max_alt = svec(0).alt;
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}

static inline void detect_descent(fu32 mode)
{
  if (mode & option(Velocity_Checks))
  {
    Require(svec(0).vel <= -DESCENT_MIN_VEL);
    Require(svec(0).alt < svec(2).alt);
  }
  else
  {
    float dh_3_0 = svec(3).alt - svec(0).alt;
    float dh_6_3 = svec(6).alt - svec(3).alt;
    Require(fabsf(dh_3_0) > fabsf(dh_6_3));
  }

  Require(svec(1).alt < svec(5).alt);
  Require(svec(4).alt < svec(7).alt);

  sm.confidence += 1;

  if (sm.confidence == 1)
  {
    extremes.max_alt = maxd(svec(4).alt, extremes.max_alt);
  }

  if (sm.confidence >= MIN_SAMP_DESCENT)
  {
    release_parachute(false);
    flight_advance(Descent);

    log_metric(id "apogee guess", (fi32) extremes.max_alt, false);
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

#ifdef SD_AVAILABLE
  sd_conclude();
#endif

  tx_thread_sleep(RECOVERY_ANNOUNCE_DELAY);
}

static inline void report_lowpass_gps(fu32 mode)
{
#ifdef GPS_AVAILABLE

  if (mode & option(GPS_Available))
  {
    f_xyz lowpass = {
      svec(0).gps.lat, svec(0).gps.lon, meas.gps.sea - LAUNCH_SITE_SEA
    };

    log_f32(SEDS_DT_GPS_DATA, 3, &lowpass);
  }

  tx_thread_sleep(RECOVERY_ANNOUNCE_DELAY / 100);

#endif
}


/* Vigilant mode */

static inline bool maybe_force(fu32 mode, float alt, state now)
{
  return now <= Armed && alt >= FLYING_ALTITUDE
                      && !within(alt - svec(3).alt, ALT_TOLER);
}

static inline bool falling(float alt, float vel, float dt, fu8 degree)
{
  bool not_vel = vel > -VIGILANT_MIN_VEL;
  bool altgain = alt > svec(degree).alt + SVHIST_ALT_TREND;
  bool midgain = alt > svec(degree / 2).alt;

  return !(not_vel || altgain || midgain);
}

static inline bool
false_positive_risk(float alt, float vel, float dt, state now)
{
  bool on_pad = now <= Armed && alt < FLYING_ALTITUDE;
  // bool ascent = now < Apogee && alt < 1750.0f;
  bool raised = now < Apogee &&
                alt > svec(STATE_HISTORY - 1).alt + SVHIST_ALT_TREND;

  return on_pad || raised;
}

static inline void vigilant_watchdog(fu32 mode, state now, float dt)
{
  bool trust_kf = sm.kf_deviations <= KF_FLUC_THRES
                  && svec(0).alt <= VIGILANT_MAX_ALT
                  && svec(0).alt >= VIGILANT_MIN_ALT;

  float alt = trust_kf ? svec(0).alt : meas.baro.alt;
  float vel = trust_kf ? svec(0).vel : (alt - svec(7).alt) /
                                       (dt * (float)(STATE_HISTORY - 1));

  if (false_positive_risk(alt, vel, dt, now))
  {
    return;
  }

  if (now < Descent && falling(alt, vel, dt, STATE_HISTORY - 1))
  {
    release_parachute(maybe_force(mode, alt, now));
    descent_initialize(mode);
    flight_advance(Descent);
    extremes.max_alt = maxd(alt, extremes.max_alt);

    now = Descent;
    tx_thread_sleep(URGENT_DEPLOYMENT_DELAY);
  }

  bool reef_window = now >= Descent && now < Reefing;
  bool reef_prereq = alt <= REEF_TARGET_ALT &&
                     (vel < 0.0f || alt < svec(4).alt);

  if (reef_window && reef_prereq && expand_parachute(false))
  {
    flight_advance(Reefing);
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

void evaluate_rocket_state(fu32 conf, float dt)
{
#ifndef NDEBUG
  kalt = svec(0).alt;
  kvel = svec(0).vel;
#endif

  state curr = current();

  if (curr < Reefing && (conf & option(Vigilant_Mode)))
  {
    vigilant_watchdog(conf, curr, dt);
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
      log_metric(id "non-evaluatable state", curr, true);
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
#endif
  }
}


/* Task */

void evaluation_entry(ULONG _)
{
  fu8 accum = 0;
  fu32 conf = load(&g_conf, Acq);
  float dt = 0;

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
      evaluate_rocket_state(conf, dt);
      sweetbench_start(3, 50, true);
    }
    else if ((accum & (Gyro_Mask | Accl_Mask)) == (Gyro_Mask | Accl_Mask))
    {
      accum |= EVALUATION_STAGED;
      accum &= ~(Gyro_Mask | Accl_Mask);

      conf = fetch_or(&g_conf,
                      option(Ascent_KF_Staged), AcqRel);

      dt = fsec(timer_exchange(AscentKF));
      ascent_predict(dt, conf);
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
    log_err(id "stack %s %u", critical, st);
    return IT_IS_NOW_OVER;
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
    log_err(id "task %s %u", critical, st);
    return IT_IS_NOW_OVER;
  }

  st = tx_event_flags_create(&eval_stage, id "E");

  if (st != TX_SUCCESS)
  {
    log_err(id "evflags %s %u", critical, st);
    return IT_IS_NOW_OVER;
  }

  kfpool_buf = _sbrk(16 + KF_POOL_SIZE);

  if (kfpool_buf == (void *)-1)
  {
    log_err(id "poolbuf %s %u", critical, st);
    return IT_IS_NOW_OVER;
  }

#ifdef PARALLEL_PREDICT_UPDATE

  st = tx_byte_pool_create(&kfpool, id "P", kfpool_buf, KF_POOL_SIZE);

  if (st != TX_SUCCESS)
  {
    log_err(id "pool %s %u", critical, st);
    return IT_IS_NOW_OVER;
  }

#endif /* PARALLEL_PREDICT_UPDATE */

  return TX_SUCCESS;
}