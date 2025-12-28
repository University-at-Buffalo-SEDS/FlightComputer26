/*
 * State monitoring and parachute deployment.
 * Designed to run as a task *in one thread*.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "FC-Threads.h"
#include "cmsis_gcc.h"
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
static rocket_t rk = {0};

/// Simple statistics struct for most recent and previous snapshots.
static stats_t stats[2] = {0};

/// Public helper. Invoked from thread context.
state_e get_rocket_state() { return rk.state; }

/// Triggers forced parachute firing and expansion with
/// compile-time specified intervals. USE WITH CAUTION.
/// Invoked from thread context.
void force_abort_deployment()
{
  rk.rec.lock = 1u;
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

/// Checks 1 to 4 new values in ring against sanity boundaries,
/// and averages valid values as defined in stats_t (deployment.h).
/// Partially valid samples are supported.
///
/// Returns DATA_NONE if ring has no new values, > 0 if some data
/// was rejected but new stats_t was formed, and < 0 if provided
/// valid data was not enough to form a complete stats_t;
static inline inference_e refresh_stats()
{
  uint_fast8_t n = atomic_exchange_explicit(&newdata, 0,
                                            memory_order_acquire);
  if (!n)
    return DATA_NONE;

  if (n > MAX_SAMPLE) {
    rk.ukf = (rk.ukf + n - MAX_SAMPLE) & UKF_RING_MASK;
    n = MAX_SAMPLE;
  }

  rk.buf = !rk.buf;

  inference_e st = DATA_OFFSET;

  float sum_vel = 0.0f;
  float sum_vax = 0.0f;

  uint_fast8_t valid_mask = 0u;
  uint_fast8_t vel_count = 1u;
  uint_fast8_t vax_count = 1u;
  uint_fast8_t k = rk.ukf;

  for (; k < n; k = (k + 1) & UKF_RING_MASK)
  {
    if (ring[k].alt < SN_MIN_ALT || ring[k].alt > SN_MAX_ALT)
    {
      st += DATA_BAD_ALT;
    }
    else if (valid_mask & VALID_ALT)
    {
      if (ring[k].alt < stats[rk.buf].min_alt)
        stats[rk.buf].min_alt = ring[k].alt;
      if (ring[k].alt > stats[rk.buf].max_alt)
        stats[rk.buf].max_alt = ring[k].alt;
    }
    else
    {
      stats[rk.buf].min_alt = ring[k].alt;
      stats[rk.buf].max_alt = ring[k].alt;
      valid_mask |= VALID_ALT;
    }
    
    if (ring[k].vel < SN_MIN_VEL || ring[k].vel > SN_MAX_VEL)
    {
      st += DATA_BAD_VEL;
    }
    else if (valid_mask & VALID_VEL)
    {
      sum_vel += ring[k].vel;
      ++vel_count;
    }
    else
    {
      sum_vel = ring[k].vel;
      valid_mask |= VALID_VEL;
    }

    if (ring[k].vax < SN_MIN_VAX || ring[k].vax > SN_MAX_VAX)
    {
      st += DATA_BAD_VAX;
    }
    else if (valid_mask & VALID_VAX)
    {
      sum_vax += ring[k].vax;
      ++vax_count;
    }
    else
    {
      sum_vax = ring[k].vax;
      valid_mask |= VALID_VAX;
    }
  }

  rk.ukf = (rk.ukf + n) & UKF_RING_MASK;

  if (!((valid_mask & VALID_ALT) &&
        (valid_mask & VALID_VEL) &&
        (valid_mask & VALID_VAX)))
  {
    /* Write to the same buffer next time. */
    rk.buf = !rk.buf;
    return st;
  }

  stats[rk.buf].avg_vel = sum_vel / (float)vel_count;
  stats[rk.buf].avg_vax = sum_vax / (float)vax_count;

  return (st == DATA_OFFSET) ? DEPL_OK : -st;
}

/// Monitors if minimum thresholds for velocity and
/// acceleration were exceded.
static inline inference_e detect_launch()
{
  if (stats[rk.buf].avg_vel >= LAUNCH_MIN_VEL &&
      stats[rk.buf].avg_vax >= LAUNCH_MIN_VAX)
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
  if (stats[rk.buf].avg_vel > stats[!rk.buf].avg_vel &&
      stats[rk.buf].min_alt > stats[!rk.buf].max_alt)
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
  if (stats[rk.buf].avg_vel >= BURNOUT_MIN_VEL &&
      stats[rk.buf].avg_vax <= BURNOUT_MAX_VAX &&
      stats[rk.buf].max_alt > stats[rk.buf].min_alt &&
      stats[rk.buf].avg_vel < stats[!rk.buf].avg_vel)
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
  if (stats[rk.buf].avg_vel <= APOGEE_MAX_VEL &&
      stats[rk.buf].avg_vel < stats[!rk.buf].avg_vel)
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
  if (stats[rk.buf].max_alt < stats[!rk.buf].min_alt &&
      stats[rk.buf].avg_vel > stats[!rk.buf].avg_vel)
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
  if (stats[rk.buf].min_alt <= REEF_TARGET_ALT && 
      stats[rk.buf].max_alt < stats[!rk.buf].min_alt)
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
  float dh = stats[rk.buf].min_alt - stats[!rk.buf].min_alt;
  float dv = stats[rk.buf].avg_vel - stats[!rk.buf].avg_vel;
  float da = stats[rk.buf].avg_vax - stats[!rk.buf].avg_vax;

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
static inline inference_e infer_rocket_state()
{
  inference_e st = refresh_stats();

  if (st == DATA_NONE) {
    rk.rec.warn = st;
    return DEPL_OK;
  } else if (st > DEPL_OK) {
    rk.rec.warn = st;
  } else if (st < DEPL_OK) {
    return st;
  }

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
/// data cache. Reassigns the rocket its active state.
static inline void try_recover_sensors()
{
  inference_e st = DEPL_OK;

  log_msg("Trying to reinitialize sensors", 31);

  __disable_irq();

  if (init_baro() != HAL_OK)
    st += DEPL_BAD_BARO;

  if (init_gyro() != HAL_OK)
    st += DEPL_BAD_GYRO;

  if (init_accel() != HAL_OK)
    st += DEPL_BAD_ACCEL;

  invalidate_dcache(); // non-IT (synchronous)

  __enable_irq();
  
  if (st == DEPL_OK) {
    log_msg("Sensor recovery successful", 27);
  } else {
    log_err("Recovery failed (code %d)", st);
  }

  /// Try to validate data anyway
  update_state(rk.rec.state);
}

/// Keeps record of successive retries,
/// and updates rocket state to Recovery or Aborted
/// if a corresponding retry threshold was exceeded.
///
/// Thresholds are configurable in deployment.h.
static inline void bad_inference_handler()
{
  ++rk.rec.ret;
  log_err("Deployment: bad inference #%u (code %d)",
          rk.rec.ret, rk.rec.inf);

  if (rk.rec.ret >= PRE_RECOV_RETRIES + PRE_ABORT_RETRIES
      && update_state(ABORTED) == DEPL_OK)
  {
    rk.rec.state = rk.state;
    log_err("FATAL: aborting deployment (%u retries)",
            rk.rec.ret);
  }
  else if (rk.state != RECOVERY &&
           rk.rec.ret >= PRE_RECOV_RETRIES &&
           rk.rec.ret % RECOVERY_INTERVAL == 0 &&
           update_state(RECOVERY) == DEPL_OK)
  {
    rk.rec.state = rk.state;
  }
}

/// Avoid crashing into the ground by turning off
/// engine and deploying parachutes.
///
/// Delays are configurable in deployment.h
static inline void deployment_abort_procedures()
{
  log_msg("Deployment: turning off engine and firing pyro", 47);
  // turn off engine here
  tx_thread_sleep(ABORT_CO2_DELAY);
  co2_high();
  tx_thread_sleep(ABORT_REEF_DELAY);
  reef_high();
}

/// Invokes state machine and checks for an error.
/// If one exists, invokes appropriate error handler
/// or logs non-critical message. Idempotent.
///
/// Idle time is configurable in FC-Threads.h.
void deployment_thread_entry(ULONG cycle)
{
  log_msg_sync("Deployment: starting thread", 28);

  if (cycle == DEPLOYMENT_THREAD_INPUT) {
    co2_low();
    reef_low();
    rk.state = IDLE;
  } else {
    rk.state = rk.rec.state;
    rk.rec.warn = DEPL_OK;
    rk.rec.ret = 0u;
  }

  while (rk.state != ABORTED) {
    if (rk.state == RECOVERY) {
      try_recover_sensors();
      continue;
    }
    
    rk.rec.inf = infer_rocket_state();

    if (rk.rec.inf == DATA_NONE) {
      tx_thread_resume(&ukf_thread);
      continue;
    } else if (rk.rec.inf < DEPL_OK) {
      bad_inference_handler();
    } else if (rk.rec.warn != DEPL_OK) {
      rk.rec.warn = DEPL_OK;
      log_err("Deployment: warn code %d", rk.rec.inf);
    } else if (rk.rec.ret > 0) {
      rk.rec.ret = 0;
      log_msg("Deployment: recovered from error", 33);
    }

    tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP);
  }

  if (rk.rec.lock || cycle >= DEPLOYMENT_THREAD_RETRIES
                            + DEPLOYMENT_THREAD_INPUT) {
    deployment_abort_procedures();

    /* Directed by            Fuad
     * Consulting Producer    Rylan
     * Executive Producer     Parth
     * Copyright              UB SEDS 2026 */
    return;
  }

  deployment_thread_entry(++cycle);
}

/// Creates a non-preemptive deployment thread with
/// defined parameters. Called manually. Provides
/// its entry point with an input of 0;
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