/* Core/Src/estimator_thread.c */

#include "platform.h"
#include "board_watchdog.h"
#include "fc_watchdog_recovery.h"
#include "fc_recovery_owner.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "FC-Threads.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"
#include "fc_estimator_status.h"
#include "ground_test.h"
#include "thread_comm.h"
#include "rud_detect.h"
#include "sensor_plausibility.h"
#include "deploy_backup.h"

#define id "EV "


TX_THREAD estimator_thread;
TX_EVENT_FLAGS_GROUP eval_stage;

void cm_align *kfpool_buf = NULL;

kf_svec sv[STATE_HISTORY] = {0};
sv_meta sm = {Startup, G_Startup, 0, 0, 0};

static stats extremes = {0};

/* The drogue timer arms only once a measured altitude has shown flight
 * (BACKUP_ARM_ALT): a false Launch detection on the pad, after a misfire,
 * must never fire a chute there. Kept across a watchdog restart. */
static uint8_t backup_armed;

#if BOARD_WATCHDOG_ENABLE
const fc_resume_region fc_resume_evaluation[] = {
  FC_RESUME_REGION(sv),
  FC_RESUME_REGION(sm),
  FC_RESUME_REGION(extremes),
  FC_RESUME_REGION(backup_armed),
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

/* One faulty sensor must not cost the chutes (sensor_plausibility.h), and
 * neither must two (deploy_backup.h, docs/rud-abort.md section 6). */
static plaus_state plaus;
static backup_dr dr;
volatile uint8_t g_fc_sensor_fallback __attribute__((used, externally_visible));
volatile uint32_t g_fc_backup_drogues __attribute__((used, externally_visible));
#define GPS_FRESH_MS 2000U
/* A GPS altitude that has not moved for this long is a held, stale fix. */
#define GPS_STUCK_MS 2000U
/* The estimator wakes at least once per barometer period (Baro_ODR_25); a
 * sensor that delivers nothing for EST_SILENT_MS is silent. */
#define EST_STEP_MS   40U
#define EST_SILENT_MS 200U

static uint32_t last_imu_ms, last_baro_ms, last_step_ms;
static float dt;   /* the last filter step's */
/* The ascent filter carried forward from its last clean barometer step by
 * the IMU alone: where it re-seats when the barometer turns implausible. */
static struct { float alt, vel; bool have, baro_was_ok; } imu_track;
static struct { float sea; uint32_t moved_ms; bool seen; } gps_watch;

static inline bool ekf_velocity_trusted(void) { return plaus_imu_ok(&plaus); }
static inline bool barometer_trusted(void) { return plaus_baro_ok(&plaus); }
/* With neither, nothing can find Apogee: the drogue timer does. On the IMU
 * alone, not before the last clean barometer state could have reached it. */
static inline bool apogee_source_trusted(void)
{
  return barometer_trusted() ||
         (ekf_velocity_trusted() && backup_apogee_possible(&dr, now_ms()));
}

static inline void note_fallback(void)
{
  g_fc_sensor_fallback = (uint8_t)((plaus_baro_ok(&plaus) ? 0U : 1U) |
                                   (plaus_velocity_ok(&plaus) ? 0U : 2U) |
                                   (plaus.imu_stuck || plaus.imu_silent ? 4U : 0U) |
                                   (dr.active ? 8U : 0U));
}

static inline void sensor_verdicts_reset(void)
{
  plaus_reset(&plaus);
  backup_armed = 0U;
  backup_dr_reset(&dr);
  last_imu_ms = last_baro_ms = last_step_ms = now_ms();
  gps_watch.seen = false;
  imu_track.have = false;
  imu_track.baro_was_ok = true;
}

/* A fresh GPS fix whose altitude still moves: GPS_FRESH_MS since the last
 * packet, and it changed within GPS_STUCK_MS (a receiver without a solution
 * that repeats its last fix is stale too). */
static inline bool gps_trusted(uint32_t t)
{
  if (!gps_watch.seen) { gps_watch.seen = true; gps_watch.sea = meas.gps.sea; gps_watch.moved_ms = t - GPS_STUCK_MS; }
  else if (meas.gps.sea != gps_watch.sea) { gps_watch.sea = meas.gps.sea; gps_watch.moved_ms = t; }
  return timer_fetch(HeartbeatRF) < GPS_FRESH_MS && (uint32_t)(t - gps_watch.moved_ms) < GPS_STUCK_MS;
}

static inline void note_chutes(uint32_t t)
{
  const fu32 c = load(&g_conf, Acq);
  backup_dr_chutes(&dr, t, c & option(Parachute_Deployed), c & option(Parachute_Expanded));
}

/* The ascent filter's velocity against the trend of its own altitude. It
 * needs the barometer: while that is implausible (the filter dead-reckons
 * on the IMU, so its altitude follows its velocity) the verdict keeps its
 * last value, and with a silent IMU there is no filter velocity. */
static inline void watch_velocity(fu32 conf, float dt)
{
  if (!(conf & option(Using_Ascent_KF)) || dt <= 0.0f) return;
  if (!barometer_trusted() || plaus.imu_silent) return;
  const float trend = (svec(0).alt - svec(STATE_HISTORY - 1).alt) /
                      (dt * (float)(STATE_HISTORY - 1));
  plaus_velocity(&plaus, svec(0).vel, trend);
  note_fallback();
}

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

/* A transition check: unless 'cond' holds, the sample is spurious for this
 * check (handle_spurious, with the condition's text) and the caller returns. */
#define RETURN_UNLESS(mode, cond)             \
  do {                                        \
    if (!(cond)) {                            \
      handle_spurious((mode), #cond);         \
      return;                                 \
    }                                         \
  } while (0)

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

  if (promotion == Launch) { timer_update(LaunchClock); fc_sd_capture_event(FC_LOG_LAUNCH_DETECTED, false); }
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
    RETURN_UNLESS(mode, fabsf(svec(0).vel) >= LAUNCH_MIN_VEL);
    RETURN_UNLESS(mode, fabsf(svec(0).vel) > fabsf(svec(2).vel));
    RETURN_UNLESS(mode, fabsf(svec(2).vel) > fabsf(svec(4).vel));
  }
  else
  {
    RETURN_UNLESS(mode, svec(0).alt >= LAUNCH_MIN_ALT);
    RETURN_UNLESS(mode, svec(0).alt > svec(2).alt);
    RETURN_UNLESS(mode, svec(2).alt > svec(4).alt);
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
    RETURN_UNLESS(mode, fabsf(svec(0).vel) >= ASCENT_MIN_VEL);
    RETURN_UNLESS(mode, fabsf(svec(0).vel) > fabsf(svec(3).vel));
  }
  else
  {
    float dh_0_3 = svec(0).alt - svec(3).alt;
    float dh_4_7 = svec(4).alt - svec(7).alt;
    RETURN_UNLESS(mode, dh_0_3 > dh_4_7);
  }

  RETURN_UNLESS(mode, svec(0).alt > svec(3).alt);
  RETURN_UNLESS(mode, svec(3).alt > svec(6).alt);

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
    RETURN_UNLESS(mode, svec(0).vel < svec(6).vel);
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
    RETURN_UNLESS(mode, within(svec(2).vel, APOGEE_MAX_VEL));
    RETURN_UNLESS(mode, within(svec(0).vel, APOGEE_MAX_VEL));
  }
  else
  {
    RETURN_UNLESS(mode, svec(0).alt > VIGILANT_MIN_APG);

    float dh_0_2 = svec(0).alt - svec(2).alt;
    float dh_2_4 = svec(2).alt - svec(4).alt;

    RETURN_UNLESS(mode, within(dh_0_2, APOGEE_ALT_TLR));
    RETURN_UNLESS(mode, within(dh_2_4, APOGEE_ALT_TLR));
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
    RETURN_UNLESS(mode, svec(0).vel <= -DESCENT_MIN_VEL);
    RETURN_UNLESS(mode, svec(0).alt < svec(2).alt);
  }
  else
  {
    float dh_3_0 = svec(3).alt - svec(0).alt;
    float dh_6_3 = svec(6).alt - svec(3).alt;
    RETURN_UNLESS(mode, fabsf(dh_3_0) > fabsf(dh_6_3));
  }

  RETURN_UNLESS(mode, svec(1).alt < svec(5).alt);
  RETURN_UNLESS(mode, svec(4).alt < svec(7).alt);

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
  RETURN_UNLESS(mode, svec(0).alt <= REEF_TARGET_ALT);
  RETURN_UNLESS(mode, svec(0).alt < svec(2).alt);
  RETURN_UNLESS(mode, svec(2).alt < svec(4).alt);

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
  RETURN_UNLESS(mode, svec(0).alt <= LANDED_MAX_ALT);
  RETURN_UNLESS(mode, within(svec(0).vel, LANDED_MAX_VEL));
  RETURN_UNLESS(mode, within(svec(4).vel, LANDED_MAX_VEL));
  RETURN_UNLESS(mode, within(dh_1, ALT_TOLER));
  RETURN_UNLESS(mode, within(dh_4, ALT_TOLER));
  RETURN_UNLESS(mode, within(dv_1, VEL_TOLER));
  RETURN_UNLESS(mode, within(dv_4, VEL_TOLER));

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
    float frame_vel = (mode & option(Velocity_Checks)) && ekf_velocity_trusted()
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
  /* An untrusted filter velocity (a bad IMU) hands over to the barometer,
   * unless the barometer is the implausible one. */
  bool trust_kf = sm.kf_deviations <= KF_FLUC_THRES
                  && svec(0).alt <= VIGILANT_MAX_ALT
                  && svec(0).alt >= VIGILANT_MIN_ALT
                  && (ekf_velocity_trusted() || !barometer_trusted());

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
    case Coast:     if (apogee_source_trusted()) detect_apogee(conf); break;
    case Apogee:    detect_descent(conf);       break;
    case Descent:   detect_reefing(conf);       break;
    case Reefing:   detect_landed(conf);        break;
    case Landed:    announce_recovery(conf);    break;
    case Recovery:  report_lowpass_gps(conf);   break;
    default:
      log_metric(id "state not evaluable", curr, true);
      return;
  }

  watch_velocity(conf, dt);

  /* Vigilant mode, and in any case while the filter velocity is untrusted:
   * then the barometer alone can still find Apogee. With neither trusted
   * the drogue timer does (backup_deployments). */
  if (curr < Reefing && ((conf & option(Vigilant_Mode)) || !ekf_velocity_trusted()) &&
      apogee_source_trusted())
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
  sensor_verdicts_reset();

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


/* The ascent update: the barometer, or IMU dead reckoning while the
 * barometer is implausible or silent ('fresh' false: no sample for
 * EST_SILENT_MS). Also run by the IGT host replay. */
void estimator_ascent_update(bool fresh)
{
  const uint32_t t = now_ms();
  bool usable = false;
  if (fresh)
  {
    usable = plaus_baro(&plaus, t, meas.baro.alt);
    last_baro_ms = t;
  }
  else plaus_baro_missing(&plaus);
  if (usable && plaus_baro_suspect(&plaus))
  {
    const float innovation = meas.baro.alt - imedsv.alt;
    usable = innovation <= PLAUS_BARO_GATE_M && innovation >= -PLAUS_BARO_GATE_M;
  }
  if (usable && plaus_baro_repeat(&plaus) && fabsf(imedsv.vel) >= PLAUS_BARO_REPEAT_MPS) usable = false;
  const float imu[6] = { meas.accl.x, meas.accl.y, meas.accl.z,
                         meas.gyro.x, meas.gyro.y, meas.gyro.z };
  plaus_imu(&plaus, imu);
  /* The IMU's own velocity change this step, from the prediction. */
  const float imu_dv = imedsv.vel - svec(1).vel;
  if (usable) ascent_update();
  else ascent_hold();
  if (imu_track.have)
  {
    imu_track.vel += imu_dv;
    imu_track.alt += imu_track.vel * dt;
  }
  /* The barometer just turned implausible: the samples it gave before the
   * verdict (up to 12 steps of a wild one) may have thrown the filter, so it
   * re-seats on the last clean step carried forward by the IMU. */
  const bool baro_ok = barometer_trusted();
  if (imu_track.baro_was_ok && !baro_ok && imu_track.have)
  {
    svec(0).alt = imedsv.alt = imu_track.alt;
    svec(0).vel = imedsv.vel = imu_track.vel;
  }
  imu_track.baro_was_ok = baro_ok;
  /* A trustworthy altitude: a clean barometer, or the IMU's dead reckoning
   * while the barometer is out and the IMU is not suspect (capped). */
  if (usable && plaus_baro_clean(&plaus))
  {
    backup_dr_trust(&dr, t, svec(0).alt, svec(0).vel, true);
    if (svec(0).alt >= BACKUP_ARM_ALT) backup_armed = 1U;
    imu_track.alt = svec(0).alt;
    imu_track.vel = svec(0).vel;
    imu_track.have = true;
  }
  else if (!usable && !barometer_trusted() && ekf_velocity_trusted())
    backup_dr_trust(&dr, t, svec(0).alt, svec(0).vel, false);
  note_fallback();
}

/* The IMU is silent: a step on the barometer alone. The velocity is the
 * altitude trend over the history; the filter velocity is untrusted, so
 * Apogee is the Vigilant check's on the barometer. */
static void estimator_ascent_baro_only(float *dt)
{
  const uint32_t t = now_ms();
  *dt = fsec(timer_exchange(AscentKF));
  const bool usable = plaus_baro(&plaus, t, meas.baro.alt) &&
                      !(plaus_baro_repeat(&plaus) && fabsf(svec(1).vel) >= PLAUS_BARO_REPEAT_MPS);
  last_baro_ms = t;
  kf_svec next = svec(1);
  if (usable)
  {
    next.alt = meas.baro.alt;
    const float span = *dt * (float)(STATE_HISTORY - 1);
    if (span > 0.0f) next.vel = (next.alt - svec(STATE_HISTORY - 1).alt) / span;
  }
  else next.alt += next.vel * *dt;
  svec(0) = next;
  imedsv = next;
  if (usable && plaus_baro_clean(&plaus))
  {
    backup_dr_trust(&dr, t, next.alt, next.vel, true);
    if (next.alt >= BACKUP_ARM_ALT) backup_armed = 1U;
  }
  note_fallback();
}

/* No trustworthy altitude: the prediction descends from the last
 * trustworthy one at the fast-biased rate (deploy_backup.h). */
static void descent_dead_reckon(uint32_t t)
{
  descent_hold();
  float vel;
  svec(0).alt = backup_dr_step(&dr, t, svec(0).alt, &vel);
  svec(0).vel = vel;
}

/* One descent step: predict, then the barometer; GPS altitude while the
 * barometer is implausible or silent and a fix is fresh and moving; else
 * the dead-reckoned prediction. Then the state machine. Also run by the
 * IGT host replay. */
void estimator_descent_step(fu32 conf, float dt, est_input in)
{
  const uint32_t t = now_ms();
  /* Descent takes every sample of a plausible barometer, ejection pulses
   * included, as it always did: the low-order descent filter absorbs single
   * steps, and the drogue timing depends on it. A GPS-only step brings no
   * barometer sample. */
  if (in == EST_BARO) { (void)plaus_baro(&plaus, t, meas.baro.alt); last_baro_ms = t; }
  else if (in == EST_SILENT) plaus_baro_missing(&plaus);
  note_chutes(t);
  /* Watched on every step, so a fix held since long ago is never new. */
  const bool gps_ok = gps_trusted(t);
  descent_predict(dt);
  /* On the ground (Landed on) every deployment is done and a resting
   * barometer may repeat itself: always take it, as before. */
  if (current() >= Landed)
  {
    if (in != EST_SILENT) descent_update();
    else descent_hold();
  }
  else if (in != EST_SILENT && plaus_baro_ok(&plaus))
  {
    descent_update();
    if (plaus_baro_clean(&plaus)) backup_dr_trust(&dr, t, svec(0).alt, svec(0).vel, true);
  }
  else if (gps_ok)
  {
    const float z[3] = { meas.gps.lon, meas.gps.lat, meas.gps.sea - LAUNCH_SITE_SEA };
    descent_update_z(z);
    backup_dr_trust(&dr, t, svec(0).alt, svec(0).vel, true);
  }
  else
  {
    descent_dead_reckon(t);
  }
  note_fallback();
  evaluate_rocket_state(conf, dt);
}

/* The drogue timer (deploy_backup.h): from Launch to Apogee, if nothing
 * has fired the drogue BACKUP_DROGUE_MS after Launch, it fires now, held
 * while a trustworthy barometer still climbs. Runs on every pass, and on
 * every wake without a sample, so a silent sensor cannot stop it. */
static void backup_deployments(void)
{
  const state s = current();
  if (s < Launch || s > Apogee || !backup_armed ||
      (load(&g_conf, Acq) & option(Parachute_Deployed))) return;
  const uint32_t t = now_ms();
  const bool climbing = barometer_trusted() && (uint32_t)(t - last_baro_ms) < EST_SILENT_MS &&
                        svec(1).alt > svec(STATE_HISTORY - 1).alt + SVHIST_ALT_TREND;
  if (!backup_drogue_due(timer_fetch(LaunchClock), climbing)) return;

  ++g_fc_backup_drogues;
  message(id "backup: no drogue by the apogee timer; firing it", true);
  thread_comm_ascent_stage(false);
  const fu32 conf = load(&g_conf, Acq);
  if (conf & option(Using_Ascent_KF)) descent_initialize(conf);
  backup_dr_measured_only(&dr);
  if (s < Apogee)
  {
    flight_advance(Apogee);
    extremes.max_alt = maxd(svec(1).alt, extremes.max_alt);
  }
  release_parachute(false, false);
  flight_advance(Descent);
}

/* The estimator's loop state: shared by its thread and the IGT host
 * replay, which plays the same events (dt: above). */
static fu8 accum;

/* The eval_stage events the next pass waits for. */
ULONG estimator_request(bool ascent)
{
  if (!ascent) return Baro_Mask | EVAL_STAGE_GPS;
  if (accum & EVALUATION_STAGED) return Baro_Mask;
  /* A silent IMU: the barometer alone drives the steps. */
  return plaus.imu_silent ? Gyro_Mask | Accl_Mask | Baro_Mask : Gyro_Mask | Accl_Mask;
}

/* A manual Deploy/Expand switched to the descent filter between passes:
 * samples offered to the ascent filter are not descent steps. */
void estimator_left_ascent(void)
{
  accum = 0;
}

/* One pass on the events got; 'done' 0: the wait timed out (EST_STEP_MS).
 * Ascent: an IMU pair stages a prediction, the next barometer sample the
 * update and the state machine; a silent barometer (EST_SILENT_MS) an
 * update without it, a silent IMU a step on the barometer alone. Descent:
 * each fresh barometer sample, a GPS fix outside Vigilant mode, or a silent
 * barometer's period runs one step. Then the drogue timer. */
void estimator_pass(bool ascent, ULONG done)
{
  const uint32_t t = now_ms();
  fu32 conf;

  if (!ascent)
  {
    /* A GPS fix while the barometer is silent is a step without it: the
     * barometer's last sample is not taken again. */
    const bool baro_silent = (uint32_t)(t - last_baro_ms) >= EST_SILENT_MS;
    const est_input in = (done & Baro_Mask) ? EST_BARO
                       : baro_silent && ((done & EVAL_STAGE_GPS) ||
                                         (uint32_t)(t - last_step_ms) >= EST_STEP_MS) ? EST_SILENT
                       : (done & EVAL_STAGE_GPS) ? EST_GPS
                       : EST_NONE;
    if (in != EST_NONE)
    {
      conf = load(&g_conf, Acq);
      dt = fsec(timer_exchange(DescentKF));
      estimator_descent_step(conf, dt, in);
      last_step_ms = t;
    }
  }
  else
  {
    accum |= (fu8)(done & (Baro_Mask | Gyro_Mask | Accl_Mask));
    const bool baro_silent = (uint32_t)(t - last_baro_ms) >= EST_SILENT_MS &&
                             (uint32_t)(t - last_step_ms) >= EST_STEP_MS;

    if ((accum & EVALUATION_STAGED) && ((accum & Baro_Mask) || baro_silent))
    {
      const bool fresh = accum & Baro_Mask;
      accum &= ~(EVALUATION_STAGED | Baro_Mask);

      estimator_ascent_update(fresh);

      thread_comm_ascent_stage(false);
      conf = load(&g_conf, Acq);
      distribution_wake(); /* it waits for the stage to clear */

      sweetbench_catch(3);
      evaluate_rocket_state(conf, dt);
      sweetbench_start(3, 50, true);
      last_step_ms = t;
    }
    else if ((accum & (Gyro_Mask | Accl_Mask)) == (Gyro_Mask | Accl_Mask))
    {
      accum |= EVALUATION_STAGED;
      accum &= ~(Gyro_Mask | Accl_Mask);
      last_imu_ms = t;
      plaus_imu_silent(&plaus, false);

      thread_comm_ascent_stage(true);
      conf = load(&g_conf, Acq);

      dt = fsec(timer_exchange(AscentKF));
      ascent_predict(dt, conf);
    }
    else if (plaus.imu_silent && (accum & Baro_Mask))
    {
      accum &= ~Baro_Mask;
      conf = load(&g_conf, Acq);
      estimator_ascent_baro_only(&dt);
      evaluate_rocket_state(conf, dt);
      last_step_ms = t;
    }
    else if (!(accum & EVALUATION_STAGED) && (uint32_t)(t - last_imu_ms) >= EST_SILENT_MS)
    {
      plaus_imu_silent(&plaus, true);
      note_fallback();
    }
  }

  backup_deployments();
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
void estimator_thread_entry(ULONG _)
{
  fu32 conf = load(&g_conf, Acq);
  bool was_ascent;

  evaluation_control_barrier();
  thread_comm_estimator_enter();
  enter_flight_mode(conf);
  fc_watchdog_checkpoint();
  was_ascent = load(&g_conf, Acq) & option(Using_Ascent_KF);
  /* A restarted thread (sensor restart, reset to Startup) starts its loop
   * afresh, as when these were its locals, and so does the stage: the
   * sensor thread offers an IMU pair first. */
  accum = 0;
  dt = 0.0f;
  thread_comm_ascent_stage(false);
  last_imu_ms = last_baro_ms = last_step_ms = now_ms();
  thread_comm_estimator_leave();

  while (!(conf & option(Eval_Abort_Flag)))
  {
    evaluation_control_barrier();
    const bool ascent = load(&g_conf, Acq) & option(Using_Ascent_KF);
    if (was_ascent && !ascent)
    {
      /* A manual Deploy/Expand switched to the descent filter between
       * passes: samples offered to the ascent filter are not descent steps. */
      estimator_left_ascent();
      (void)tx_event_flags_set(&eval_stage,
                               ~(ULONG)(Baro_Mask | Gyro_Mask | Accl_Mask | EVAL_STAGE_GPS),
                               TX_AND);
    }
    was_ascent = ascent;

    const ULONG request = estimator_request(ascent);
    ULONG done = 0;

    /* At least once per barometer period, so a silent sensor still gets
     * its steps and the drogue timer its checks. */
    const UINT got = tx_event_flags_get(&eval_stage, request | EVAL_STAGE_WAKE,
                                        TX_OR_CLEAR, &done, EST_STEP_MS);
    if (got == TX_NO_EVENTS) done = 0;
    else if (got != TX_SUCCESS) continue;

    evaluation_control_barrier();
    /* One pass (a predict, or an update and the state machine) never
     * interleaves a recovery command such as a manual Deploy/Expand. */
    thread_comm_estimator_enter();
    if (ascent != (bool)(load(&g_conf, Acq) & option(Using_Ascent_KF)))
    {
      /* The filter changed while this thread waited for the guard: decide
       * again on the current mode. */
      thread_comm_estimator_leave();
      continue;
    }

    estimator_pass(ascent, done & request);

    /* A switch made by this pass (Apogee) keeps the samples offered since. */
    was_ascent = load(&g_conf, Acq) & option(Using_Ascent_KF);
    conf = load(&g_conf, Acq);
    thread_comm_estimator_leave();
  }
}

UINT create_estimator_thread(TX_BYTE_POOL *byte_pool)
{
  UINT st;
  CHAR *pointer;

  const char *critical = "creation failure:";

  st = tx_byte_allocate(byte_pool, (VOID **)&pointer,
                        ESTIMATOR_STACK_BYTES, TX_NO_WAIT);

  if (st != TX_SUCCESS)
  {
    log_err(id "stack %s %u", critical, st);
    return FC_INIT_FAILED;
  }

  st = tx_thread_create(&estimator_thread,
                        "Estimator",
                        estimator_thread_entry,
                        ESTIMATOR_INPUT,
                        pointer,
                        ESTIMATOR_STACK_BYTES,
                        ESTIMATOR_PRIORITY,
                        /* No preemption threshold: thread_comm's estimator
                         * guard keeps recovery out of a pass. */
                        ESTIMATOR_PRIORITY,
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