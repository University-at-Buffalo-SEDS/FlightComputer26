/* Core/Src/evaluation.c */

#include "platform.h"
#include "board_watchdog.h"
#include "fc_watchdog_recovery.h"
#include "fc_recovery_owner.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"
#include "fc_estimator_status.h"
#include "ground_test.h"
#include "thread_comm.h"
#include "rud_detect.h"

#define id "EV "


TX_THREAD evaluation_task;
TX_EVENT_FLAGS_GROUP eval_stage;

void cm_align *kfpool_buf = NULL;

kf_svec sv[STATE_HISTORY] = {0};
sv_meta sm = {Startup, G_Startup, 0, 0, 0};

static stats extremes = {0};

#if BOARD_WATCHDOG_ENABLE
const fc_resume_region fc_resume_evaluation[] = {
  FC_RESUME_REGION(sv),
  FC_RESUME_REGION(sm),
  FC_RESUME_REGION(extremes),
  { NULL, 0U }
};
#endif


#ifndef NDEBUG
uncached volatile float kalt, kvel;
#endif


/* Break-up (RUD) detector, rud_detect.h: armed from Launch until Apogee,
 * while the ascent filter (and so the IMU) runs. On a detection the
 * estimator stops itself exactly as an operator Evaluation_Abort would: no
 * automatic drogue or main after it; the operator's manual Deploy/Expand
 * stay available. */
static rud_state rud;
volatile uint32_t g_fc_rud_aborts __attribute__((used, externally_visible));

static inline void watch_for_breakup(fu32 conf, state curr)
{
  const rud_input in = {
    .t_ms = now_ms(),
    .armed = curr >= Launch && curr <= Coast && (conf & option(Using_Ascent_KF)),
    .gyro = { meas.gyro.x, meas.gyro.y, meas.gyro.z },
    .accel = { meas.accl.x, meas.accl.y, meas.accl.z },
    .baro_alt = meas.baro.alt,
  };
  if (rud_step(&rud, &in))
  {
    ++g_fc_rud_aborts;
    message(id "RUD: break-up detected; automatic deployment stopped", true);
    fc_recovery_request_eval_stop();
  }
}

/* Estimator status (fc_estimator_status.h) */

volatile uint8_t g_fc_est_phase;
volatile uint32_t g_fc_est_cycles, g_fc_est_start;
static uint32_t est_run;

static inline void estimator_cycle(float probe)
{
  /* -ffast-math folds isfinite(); test the exponent bits instead. */
  union { float value; uint32_t bits; } cast = {.value = probe};
  ++g_fc_est_cycles;
  if (!g_fc_est_start) { g_fc_est_start = now_ms() | 1U; est_run = 0; }
  g_fc_est_phase = (cast.bits & 0x7f800000U) == 0x7f800000U ||
                   sm.kf_deviations > KF_FLUC_THRES ? FC_EST_DIVERGED
                 : ++est_run >= FC_EST_CONVERGE_CYCLES ? FC_EST_CONVERGED
                 : FC_EST_RUNNING;
}


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

  const uint32_t reset_mask = __get_PRIMASK(); __disable_irq();
  fc_recovery_ota_reset_retire_locked();
  const bool wrong_sequence = fetch_add(&sm.flight, 1, AcqRel) != promotion - 1;
  if (wrong_sequence) store(&sm.flight, promotion, Rel);
  __set_PRIMASK(reset_mask);
  if (wrong_sequence) message(id "enter vigilant mode", false);

  if (promotion == Launch) fc_sd_capture_event(FC_LOG_LAUNCH_DETECTED, false);
  if (promotion == Landed) sd_launch_landed(now_ms());
#if FC_GROUND_TEST
  if (promotion == Landed) igt_landed();
#endif
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
    thread_comm_estimator_sleep(LAUNCH_CONFIRM_DELAY);
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
    thread_comm_estimator_sleep(APOGEE_CONFIRM_DELAY);
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
    release_parachute(false, false);
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
    expand_parachute(false, false);
    flight_advance(Reefing);
  }
}

static uint32_t landed_ms;

static inline void detect_landed(fu32 mode)
{
  float dh_1 = svec(0).alt - svec(1).alt;
  float dv_1 = svec(0).vel - svec(1).vel;
  float dh_4 = svec(2).alt - svec(6).alt;
  float dv_4 = svec(2).vel - svec(6).vel;

  /* Steady descent under the main also has near-constant altitude steps and
   * velocity; only a resting vehicle near the pad baseline is on the ground. */
  Require(svec(0).alt <= LANDED_MAX_ALT);
  Require(within(svec(0).vel, LANDED_MAX_VEL));
  Require(within(svec(4).vel, LANDED_MAX_VEL));
  Require(within(dh_1, ALT_TOLER));
  Require(within(dh_4, ALT_TOLER));
  Require(within(dv_1, VEL_TOLER));
  Require(within(dv_4, VEL_TOLER));

  sm.confidence += 1;

  if (sm.confidence >= MIN_SAMP_LANDED)
  {
    flight_advance(Landed);
    landed_ms = now_ms();
  }
}

/* Wait out the announce delay without sleeping the estimator, so the descent
 * filter keeps stepping on every barometer sample. The SD writer keeps
 * logging through Landed and Recovery (sdpipeline.c). */
static inline void announce_recovery(fu32 mode)
{
  if ((uint32_t)(now_ms() - landed_ms) < RECOVERY_ANNOUNCE_DELAY) return;
  flight_advance(Recovery);
  message(id "recovery announced", true);
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

  thread_comm_estimator_sleep(RECOVERY_ANNOUNCE_DELAY / 1000);

#endif
}


/* Vigilant mode */

static inline conditional bool maybe_force(fu32 mode, float alt, state now)
{
  return now <= Armed && alt >= FLYING_ALTITUDE
                      && !within(alt - svec(3).alt, ALT_TOLER);
}

static inline bool falling(float alt, float vel, float dt, fu8 degree)
{
  bool insufficient_vel = vel > -VIGILANT_MIN_VEL;
  bool gained_altitude = alt > svec(degree).alt + SVHIST_ALT_TREND;
  bool mid_gained_alt = alt > svec(degree / 2).alt;

  return !(insufficient_vel || gained_altitude || mid_gained_alt);
}

static inline bool
false_positive_risk(float alt, float vel, float dt, state now, fu32 mode)
{
  bool insufficient_alt = alt < FLYING_ALTITUDE;

  if (now >= Apogee)
  {
    return insufficient_alt;
  }

  float avg_new_alt = (alt + svec(1).alt + svec(2).alt + svec(3).alt) / 4.0f;
  float avg_old_alt = (svec(4).alt + svec(5).alt + svec(6).alt + svec(7).alt) / 4.0f;

  bool gained_altitude = avg_new_alt > (avg_old_alt - SVHIST_ALT_TREND);

  fu8 ascent_samples = 0;
  float avg_vel = 0.0f;

  for (fu8 k = 1; k < STATE_HISTORY; ++k)
  {
    float frame_vel = mode & option(Velocity_Checks)
                    ? svec(k - 1).vel
                    : (svec(k - 1).alt - svec(k).alt) / dt;

    avg_vel += frame_vel;

    if (frame_vel > -VEL_TOLER)
    {
      ++ascent_samples;
    }
  }

  avg_vel /= (float)(STATE_HISTORY - 1);

  bool high_velocity = avg_vel > -DESCENT_MIN_VEL || ascent_samples >= 2;

  return insufficient_alt || high_velocity || gained_altitude;
}

static inline void vigilant_watchdog(fu32 mode, state now, float dt)
{
  bool trust_kf = sm.kf_deviations <= KF_FLUC_THRES
                  && svec(0).alt <= VIGILANT_MAX_ALT
                  && svec(0).alt >= VIGILANT_MIN_ALT;

  float alt = trust_kf ? svec(0).alt : meas.baro.alt;
  float vel = trust_kf ? svec(0).vel : (alt - svec(7).alt) /
                                       (dt * (float)(STATE_HISTORY));

  if (false_positive_risk(alt, vel, dt, now, mode))
  {
    return;
  }

  /* State correction only: hand over to detect_descent and detect_reefing,
   * which own the drogue and main deployments. Advancing past Apogee here
   * would skip both of them. */
  if (now < Apogee && falling(alt, vel, dt, STATE_HISTORY - 1))
  {
    descent_initialize(mode);
    flight_advance(Apogee);
    extremes.max_alt = maxd(alt, extremes.max_alt);
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

#if FC_GROUND_TEST
  /* Outputs computed from replayed inputs go out only as SIM_SAMPLE. */
  if (igt_sim_running())
  {
    igt_publish(conf & option(Using_Ascent_KF) ? IGT_KIND_ASCENT : IGT_KIND_DESCENT,
                tmp, elements, now_ms());
  }
  else
#endif
  {
#ifdef SD_AVAILABLE
    /* Preserve each completed filter output independently of radio/report
     * throttling; the bounded SD path accounts for any storage refusal. */
    sd_append_f32(kind_sd, tmp, elements);
#endif

    if (timer_probe_ground(KFRemote, rates.gnd))
    {
      log_periodic_f32(kind_gnd, elements, tmp);
    }
  }

  estimator_cycle(svec(0).alt + svec(0).vel);
  sm.idx = (sm.idx + 1) & STATE_HISTORY_MASK;
}

void evaluate_rocket_state(fu32 conf, float dt)
{
#ifndef NDEBUG
  kalt = svec(0).alt;
  kvel = svec(0).vel;
#endif

#if BOARD_WATCHDOG_ENABLE
  static uint8_t resume_history;
  /* Fill the complete history with fresh sensor cycles before the FSM can
   * act on comparisons or successive-sample confidence after an outage. */
  if (g_fc_watchdog_resumed && resume_history < STATE_HISTORY) {
    ++resume_history;
    goto publish;
  }
#endif

  state curr = current();

  watch_for_breakup(conf, curr);

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
      log_metric(id "state not evaluable", curr, true);
      return;
  }

  if (curr < Reefing && (conf & option(Vigilant_Mode)))
  {
    vigilant_watchdog(conf, curr, dt);
  }

#if BOARD_WATCHDOG_ENABLE
publish:
#endif
  propel_kalman_state(conf);
  fc_watchdog_checkpoint();
  board_watchdog_progress(BOARD_WATCHDOG_SAFETY);
}

static inline void enter_flight_mode(fu32 conf)
{
  static bool restored_entry;
  g_fc_est_phase = FC_EST_INIT;
  g_fc_est_start = 0;
  if (g_fc_watchdog_resumed && !restored_entry) { restored_entry = true; return; }

  if (conf & option(Launch_Requested))
  {
    sm.idx = (sm.idx - 1) & STATE_HISTORY_MASK;
    message(id "flight resumed", true);
  }
  else
  {
    ascent_initialize(conf);
    message(id "launch received", true);
    fetch_or(&g_conf, option(Launch_Requested), Rel);
    distribution_wake();

#ifdef DESCENT_TEST
    descent_initialize(conf);
{
      const uint32_t reset_mask = __get_PRIMASK(); __disable_irq();
      fc_recovery_ota_reset_retire_locked();
      store(&sm.flight, Apogee, Rel);
      __set_PRIMASK(reset_mask);
    }
#endif
  }
}


/* Task */

static void evaluation_control_barrier(void)
{
  while (fc_recovery_eval_pause_requested()) {
    g_fc_est_phase = FC_EST_PAUSED;
    fc_recovery_evaluation_quiescent();
    /* A failed or prematurely lifted suspend cannot consume synthetic wake
     * flags as a measurement, nor spin and starve the telemetry commit. */
    if (fc_recovery_eval_pause_requested()) tx_thread_sleep(1U);
  }
  fc_recovery_evaluation_running();
}

/* The estimator: the ascent EKF and the descent filter and, on every
 * filter step, the flight state machine. Ascent: an IMU pair stages a
 * prediction, the next barometer sample the update and the state machine
 * (the distribution thread offers them in that order through eval_stage).
 * Descent: each fresh barometer sample, or GPS fix outside Vigilant mode,
 * runs one predict/update and the state machine. */
void evaluation_entry(ULONG _)
{
  fu8 accum = 0;
  fu32 conf = load(&g_conf, Acq);
  float dt = 0;
  bool was_ascent;

  evaluation_control_barrier();
  thread_comm_estimator_enter();
  enter_flight_mode(conf);
  fc_watchdog_checkpoint();
  was_ascent = load(&g_conf, Acq) & option(Using_Ascent_KF);
  thread_comm_estimator_leave();

  while (!(conf & option(Eval_Abort_Flag)))
  {
    evaluation_control_barrier();
    const bool ascent = load(&g_conf, Acq) & option(Using_Ascent_KF);
    if (was_ascent && !ascent)
    {
      /* A manual Deploy/Expand switched to the descent filter between
       * passes: samples offered to the ascent filter are not descent steps. */
      accum = 0;
      (void)tx_event_flags_set(&eval_stage,
                               ~(ULONG)(Baro_Mask | Gyro_Mask | Accl_Mask | EVAL_STAGE_GPS),
                               TX_AND);
    }
    was_ascent = ascent;

    ULONG done, request = !ascent ? Baro_Mask | EVAL_STAGE_GPS
                        : accum & EVALUATION_STAGED ? Baro_Mask
                        : Gyro_Mask | Accl_Mask;

    if (tx_event_flags_get(&eval_stage, request | EVAL_STAGE_WAKE,
                           TX_OR_CLEAR, &done,
                           TX_WAIT_FOREVER) != TX_SUCCESS)
    { continue; }

    evaluation_control_barrier();
    /* One pass (a predict, or an update and the state machine) never
     * interleaves a recovery command such as a manual Deploy/Expand. */
    thread_comm_estimator_enter();
    if ((done & request) == 0U ||
        ascent != (bool)(load(&g_conf, Acq) & option(Using_Ascent_KF)))
    {
      /* A mode wake, or the filter changed while this thread waited for the
       * guard: decide again on the current mode. */
      thread_comm_estimator_leave();
      continue;
    }

    if (!ascent)
    {
      conf = load(&g_conf, Acq);
      dt = fsec(timer_exchange(DescentKF));
      descent_predict(dt);
      descent_update();
      evaluate_rocket_state(conf, dt);
    }
    else
    {
      accum |= (fu8)(done & (Baro_Mask | Gyro_Mask | Accl_Mask));

      if ((accum & EVALUATION_STAGED) && (accum & Baro_Mask))
      {
        accum &= ~(EVALUATION_STAGED | Baro_Mask);

        ascent_update();

        conf = fetch_and(&g_conf,
                         ~option(Ascent_KF_Staged), AcqRel);
        distribution_wake(); /* it waits for the stage to clear */

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
    /* A switch made by this pass (Apogee) keeps the samples offered since. */
    was_ascent = load(&g_conf, Acq) & option(Using_Ascent_KF);
    thread_comm_estimator_leave();
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
    return FC_INIT_FAILED;
  }

  st = tx_thread_create(&evaluation_task,
                        "Evaluation",
                        evaluation_entry,
                        EVAL_INPUT,
                        pointer,
                        EVAL_STACK_BYTES,
                        EVAL_PRIORITY,
                        /* No preemption threshold: thread_comm's estimator
                         * guard keeps recovery out of a pass. */
                        EVAL_PRIORITY,
                        TX_NO_TIME_SLICE,
                        TX_DONT_START);

  if (st != TX_SUCCESS)
  {
    log_err(id "task %s %u", critical, st);
    return FC_INIT_FAILED;
  }

  st = tx_event_flags_create(&eval_stage, id "E");

  if (st != TX_SUCCESS)
  {
    log_err(id "evflags %s %u", critical, st);
    return FC_INIT_FAILED;
  }

  kfpool_buf = _sbrk(16 + KF_POOL_SIZE);

  if (kfpool_buf == (void *)-1)
  {
    log_err(id "poolbuf %s %u", critical, st);
    return FC_INIT_FAILED;
  }

  return TX_SUCCESS;
}