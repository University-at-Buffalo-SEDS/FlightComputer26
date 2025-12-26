/*
 * State monitoring and parachute deployment.
 * Designed to run as a task *in one thread*.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "platform.h"
#include "deployment.h"
#include "ukf.h"

/// Aliased ThreadX thread and thread stack.
TX_THREAD deployment_thread;
ULONG deployment_thread_stack[DEPLOYMENT_THREAD_STACK_SIZE
                              / sizeof(ULONG)];

/// Counter for the amount of new records in filter ring.
extern atomic_uint_fast8_t newdata;
extern filter_t ring[];

/// Contains most-often used globals
static rket_t rk = {0};

/// Most-recent and previous data buffers stored together
static filter_t data[2][MAX_SAMPLE] = {0};

/// Simple statistics struct for each buffer.
static stats_t stats[2] = {0};

/// Public helper. Invoked from thread context.
state_e get_rket_state() { return rk.state; }

/// Triggers forced parachute firing and expansion with
/// compile-time specified intervals. USE WITH CAUTION.
/// Invoked from thread context.
void force_abort_deployment()
{
  rk.rec.lock = 1;
  /// The lock must be "force acquired" before writing state
  /// to prevent state transitions on last remaining iteration.
  __DMB();
  rk.state = ABORTED;
}

/// Safe helper that respects abortion lock.
static inline inference_e update_state(state_e s)
{
  if (rk.rec.lock) return DEPL_DOOM;

  rk.state = s;
  return DEPL_OK;
}

/// This is ravings of a madman but required in order
/// to be able to accept and filter partially valid samples.
static inline inference_e refresh_data()
{
  inference_e st = DEPL_OK;
  uint_fast8_t n = atomic_exchange_explicit(&newdata, 0,
                                            memory_order_acq_rel);
  if (!n)
    return DEPL_NO_INPUT;

  if (n > MAX_SAMPLE) {
    rk.i.ex = (rk.i.ex + n - MAX_SAMPLE) & UKF_RING_MASK;
    n = MAX_SAMPLE;
  }

  rk.i.a = !rk.i.a;
  rk.i.sp = rk.i.sc;
  ++rk.i.ex;

  if (ring[rk.i.ex].alt < SN_MIN_ALT ||
      ring[rk.i.ex].alt > SN_MAX_ALT) {
    st += DEPL_BAD_ALT;
    stats[rk.i.a].min_alt = 0.0f;
  } else {
    stats[rk.i.a].min_alt = ring[rk.i.ex].alt;
    stats[rk.i.a].max_alt = ring[rk.i.ex].alt;
  }

  float sum_vel = 0.0f;
  float sum_vax = 0.0f;
  uint_fast8_t valid_vel, valid_vax = 0u;
  
  if (ring[rk.i.ex].vel < SN_MIN_VEL ||
    ring[rk.i.ex].vel > SN_MAX_VEL) {
    st += DEPL_BAD_VEL;
  } else {
    sum_vel = ring[rk.i.ex].vel;
    valid_vel = 1u;
  }

  if (ring[rk.i.ex].vax < SN_MIN_VAX ||
    ring[rk.i.ex].vax > SN_MAX_VAX) {
    st += DEPL_BAD_VAX;
  } else {
    sum_vax += ring[rk.i.ex].vax;
    valid_vax = 1u;
  }

  for (rk.i.sc = 1; rk.i.sc < n; ++rk.i.sc)
  {
    uint_fast8_t j = (rk.i.ex + rk.i.sc) & UKF_RING_MASK;

    if (ring[j].alt < SN_MIN_ALT || ring[j].alt > SN_MAX_ALT) {
      st += DEPL_BAD_ALT;
    } else {
      if (ring[j].alt < stats[rk.i.a].min_alt)
        stats[rk.i.a].min_alt = ring[j].alt;
      if (ring[j].alt > stats[rk.i.a].max_alt)
        stats[rk.i.a].max_alt = ring[j].alt;
    }
    
    if (ring[j].vel < SN_MIN_VEL || ring[j].vel > SN_MAX_VEL) {
      st += DEPL_BAD_VEL;
    } else {
      sum_vel += ring[j].vel;
      ++valid_vel;
    }

    if (ring[j].vax < SN_MIN_VAX || ring[j].vax > SN_MAX_VAX) {
      st += DEPL_BAD_VAX;
    } else {
      sum_vax += ring[j].vax;
      ++valid_vax;
    }
  }

  if (!stats[rk.i.a].min_alt || !valid_vel || !valid_vax) {
    return st;
  }

  stats[rk.i.a].avg_vel = sum_vel / (float)valid_vel;
  stats[rk.i.a].avg_vax = sum_vax / (float)valid_vax;

  return -st;
}

/// Monitors if minimum thresholds for velocity and
/// acceleration were exceded.
static inline inference_e detect_launch()
{
  if (stats[rk.i.a].avg_vel >= LAUNCH_MIN_VEL &&
      stats[rk.i.a].avg_vax >= LAUNCH_MIN_VAX)
  {
    if (update_state(LAUNCH) == DEPL_OK) {
      rk.samp.ascent = 0;
      log_msg("Launch detected", 16);
      tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
    }
  }

  return DEPL_OK;
}

/// Monitors height and velocity increase consistency.
static inline inference_e detect_ascent()
{
  if (stats[rk.i.a].avg_vel > stats[!rk.i.a].avg_vel &&
      stats[rk.i.a].min_alt > stats[!rk.i.a].max_alt)
  {
    ++rk.samp.ascent;
    if (rk.samp.ascent >= MIN_SAMP_ASCENT
        && update_state(ASCENT) == DEPL_OK)
    {
      rk.samp.burnout = 0;
      log_msg("Launch confirmed", 17);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rk.samp.ascent > 0)
  {
    rk.samp.ascent = 0;
    return DEPL_N_LAUNCH;
  }
#endif

  return DEPL_OK;
}

/// Monitors if minimum threshold for velocity and
/// maximum threshold for acceleration were passed.
/// Checks for height increase and velocity decrease
/// consistency.
static inline inference_e detect_burnout()
{
  if (stats[rk.i.a].avg_vel >= BURNOUT_MIN_VEL &&
      stats[rk.i.a].avg_vax <= BURNOUT_MAX_VAX &&
      stats[rk.i.a].max_alt > stats[rk.i.a].min_alt &&
      stats[rk.i.a].avg_vel < stats[!rk.i.a].avg_vel)
  {
    ++rk.samp.burnout;
    if (rk.samp.burnout >= MIN_SAMP_BURNOUT
        && update_state(BURNOUT) == DEPL_OK)
    {
      log_msg("Watching for apogee", 20);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rk.samp.burnout > 0)
  {
    rk.samp.burnout = 0;
    return DEPL_N_BURNOUT;
  }
#endif

  return DEPL_OK;
}

/// Initially monitors for continuing burnout and
/// for velocity to pass the minimum threshold.
static inline inference_e detect_apogee()
{
  if (stats[rk.i.a].avg_vel <= APOGEE_MAX_VEL &&
      stats[rk.i.a].avg_vel < stats[!rk.i.a].avg_vel)
  {
    if (update_state(APOGEE) == DEPL_OK) {
      rk.samp.descent = 0;
      log_msg("Approaching apogee", 19);
      tx_thread_sleep(APOGEE_CONFIRM_DELAY);
    }
  }

  return DEPL_OK;
}

/// Monitors for decreasing altitude and increasing velocity.
static inline inference_e detect_descent()
{
  if (stats[rk.i.a].max_alt < stats[!rk.i.a].min_alt &&
      stats[rk.i.a].avg_vel > stats[!rk.i.a].avg_vel)
  {
    ++rk.samp.descent;
    if (rk.samp.descent >= MIN_SAMP_DESCENT
        && update_state(DESCENT) == DEPL_OK)
    {
      rk.samp.landing = 0;
      co2_high();
      log_msg("Fired pyro, descending", 23);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rk.samp.descent > 0)
  {
    rk.samp.descent = 0;
    return DEPL_N_DESCENT;
  }
#endif

  return DEPL_OK;
}

/// Monitors for falling below a specific altitude,
/// and checks for altitude consistency.
static inline inference_e detect_reef()
{
  if (stats[rk.i.a].min_alt <= REEF_TARGET_ALT && 
      stats[rk.i.a].max_alt < stats[!rk.i.a].min_alt)
  {
    ++rk.samp.landing;
    if (rk.samp.landing >= MIN_SAMP_REEF
        && update_state(REEF) == DEPL_OK)
    {
      rk.samp.idle = 0;
      reef_low();
      log_msg("Expanded parachute", 19);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rk.samp.landing > 0)
  {
    rk.samp.landing = 0;
    return DEPL_N_REEF;
  }
#endif

  return DEPL_OK;
}

/// Monitors all statistical metrics to not deviate
/// beyond allowed tolerance thresholds.
static inline inference_e detect_landed()
{
  float dh = stats[rk.i.a].min_alt - stats[!rk.i.a].min_alt;
  float dv = stats[rk.i.a].avg_vel - stats[!rk.i.a].avg_vel;
  float da = stats[rk.i.a].avg_vax - stats[!rk.i.a].avg_vax;

  if ((dh <= ALT_TOLER && dh >= -ALT_TOLER) &&
      (dv <= VEL_TOLER && dv >= -VEL_TOLER) &&
      (da <= VAX_TOLER && da >= -VAX_TOLER))
  {
    ++rk.samp.idle;
    if (rk.samp.idle >= MIN_SAMP_LANDED
        && update_state(LANDED) == DEPL_OK)
    {
      log_msg("Rocket landed", 14);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rk.samp.idle > 0)
  {
    rk.samp.idle = 0;
    return DEPL_N_LANDED;
  }
#endif

  return DEPL_OK;
}

/// The state machine does not allow state regression
/// and triggers checks to prevent premature transitions.
static inline inference_e infer_rket_state()
{
  inference_e st;

  if ((st = refresh_data()) != DEPL_OK)
    return st;

  switch (rk.state) {
    case IDLE:
      return detect_launch();
    case LAUNCH:
      return detect_ascent();
    case ASCENT:
      return detect_burnout();
    case BURNOUT:
      return detect_apogee();
    case APOGEE:
      return detect_descent();
    case DESCENT:
      return detect_reef();
    case REEF:
      return detect_landed();
    case LANDED:
      return DEPL_OK;
    default:
      return DEPL_DOOM;
  }
}

/// Tries to reinitializes sensors and invalidate
/// data cache. Reassigns the rket its active state.
static inline void try_recover_sensors()
{
  inference_e st = DEPL_OK;

  log_msg("Trying to recover sensors", 26);

  __disable_irq();

  if (init_baro() != HAL_OK)
    st += DEPL_BAD_BARO;

  if (init_gyro() != HAL_OK)
    st += DEPL_BAD_GYRO;

  // if (init_accel() != HAL_OK)
  //   st += DEPL_BAD_ACCEL;

  invalidate_dcache();

  __disable_irq();
  
  if (st == DEPL_OK) {
    log_msg("Sensor recovery successful", 27);
  } else {
    log_err("Recovery failed (code %d)", st);
  }

  /// Try to validate data anyway
  update_state(rk.rec.state);
}

/// Keeps record of successive retries,
/// and updates rket state to Recovery or Aborted
/// if a corresponding retry threshold was exceeded.
///
/// Thresholds are configurable in deployment.h.
static inline void bad_inference_handler()
{
  ++rk.rec.ret;
  log_err("Deployment: bad inference #%u (code %d)",
          rk.rec.ret, rk.rec.inf);

  if (rk.rec.ret >= PRE_RECOV_RETRIES + PRE_ABORT_RETRIES)
  {
    /// Return value ignored - aborting anyway :D
    update_state(ABORTED);
    log_err("FATAL: aborting deployment (%u retries)",
            rk.rec.ret);
  }
  else if (rk.state != RECOVERY &&
           rk.rec.ret >= PRE_RECOV_RETRIES &&
           rk.rec.ret % RECOVERY_INTERVAL == 0)
  {
    rk.rec.state = rk.state;
    update_state(RECOVERY);
  }
}

/// Avoid crashing into the ground by turning off
/// engine and deploying parachutes.
///
/// Delays are configurable in deployment.h
static inline void deployment_abort_procedures()
{
  log_msg("Deployment: turning off engine and firing pyro", 47);
  // turn off engine
  tx_thread_sleep(ABORT_CO2_DELAY);
  co2_high();
  tx_thread_sleep(ABORT_REEF_DELAY);
  reef_high();
}

/// Invokes state machine and checks for an error.
/// If one exists, invokes appropriate error handler
/// or logs non-critical message.
///
/// Idle time is configurable in FC-Threads.h.
void deployment_thread_entry(ULONG input)
{
  (void)input;

  log_msg_sync("Deployment: starting thread", 28);

  co2_low();
  reef_low();
  rk.state = IDLE;

  while (rk.state != ABORTED) {
    if (rk.state == RECOVERY) {
      try_recover_sensors();
      continue;
    }
    
    rk.rec.inf = infer_rket_state();

    if (rk.rec.inf == DEPL_NO_INPUT) {
      // tx_thread_resume(&kalman_thread);
      continue;
    } else if (rk.rec.inf < DEPL_OK) {
      bad_inference_handler();
    } else if (rk.rec.inf > DEPL_OK) {
      log_err("Deployment: warn code %d", rk.rec.inf);
    } else if (rk.rec.ret > 0) {
      rk.rec.ret = 0;
      log_msg("Deployment: recovered from error", 33);
    }

    tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP);
  }

  if (rk.state == ABORTED)
    deployment_abort_procedures();
}

/// Creates a non-preemptive deployment thread with
/// defined parameters. Called manually. Provides
/// its entry point with a throwaway input.
///
/// Stack size and priority are configurable in FC-Threads.h.
void create_deployment_thread(void)
{
  UINT st = tx_thread_create(&deployment_thread,
                             "Deployment Thread",
                             deployment_thread_entry,
                             DEPLOYMENT_THREAD_INPUT,
                             deployment_thread_stack,
                             DEPLOYMENT_THREAD_STACK_SIZE,
                             DEPLOYMENT_THREAD_PRIORITY,
                             /* No preemption */
                             DEPLOYMENT_THREAD_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("Failed to create deployment thread: %u", (unsigned)st);
  }
}