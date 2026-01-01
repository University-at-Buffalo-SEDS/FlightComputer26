/*
 * State monitoring and parachute deployment.
 * Designed to run as a task *in one thread*.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "deployment.h"

TX_THREAD deployment_thread;
ULONG deployment_thread_stack[DEPLOYMENT_THREAD_STACK_SIZE
                              / sizeof(ULONG)];

/// From UKF (ukf.c).
extern atomic_uint_fast8_t newdata;
extern filter_t ring[];

/// Rocket controls and statistics.
static flight_t rock = {0};
static stats_t stats[2] = {0};


/* ------ Public API ------ */

/// Does not cover states before rocket is ready to launch.
state_e current_flight_state() { return rock.state; }

/// Send enum-specified command to deployment.
/// First command should be 'ABORT' to enter manual mode.
void deployment_send_command(command_e command)
{
  if (command == ABORT)
    rock.abort |= MANUAL_ABORT;
  else
    rock.cmd = command;
}


/* ------ Data acquisition ------ */

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
  if (!n) {
    return DATA_NONE;
  } else if (n > MAX_SAMPLE) {
    rock.ukf = (rock.ukf + n - MAX_SAMPLE) & UKF_RING_MASK;
    n = MAX_SAMPLE;
  }

  rock.buf = !rock.buf;

  inference_e st = DATA_OFFSET;

  float sum_vel = 0.0f;
  float sum_vax = 0.0f;

  uint_fast8_t valid_mask = 0u;
  uint_fast8_t vel_count = 1u;
  uint_fast8_t vax_count = 1u;

  for (uint_fast8_t k = rock.ukf; k < n; k = (k + 1) & UKF_RING_MASK)
  {
    if (ring[k].alt < SN_MIN_ALT || ring[k].alt > SN_MAX_ALT)
    {
      st += DATA_BAD_ALT;
    }
    else if (valid_mask & VALID_ALT)
    {
      if (ring[k].alt < stats[rock.buf].min_alt)
        stats[rock.buf].min_alt = ring[k].alt;
      if (ring[k].alt > stats[rock.buf].max_alt)
        stats[rock.buf].max_alt = ring[k].alt;
    }
    else
    {
      stats[rock.buf].min_alt = ring[k].alt;
      stats[rock.buf].max_alt = ring[k].alt;
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

  rock.ukf = (rock.ukf + n) & UKF_RING_MASK;

  if (!((valid_mask & VALID_ALT) &&
        (valid_mask & VALID_VEL) &&
        (valid_mask & VALID_VAX)))
  {
    /* Write to the same buffer next time. */
    rock.buf = !rock.buf;
    return st;
  }

  stats[rock.buf].avg_vel = sum_vel / (float)vel_count;
  stats[rock.buf].avg_vax = sum_vax / (float)vax_count;

  return (st == DATA_OFFSET) ? DEPL_OK : -st;
}


/* ------ State machine ------ */

/// Monitors if minimum thresholds for velocity and
/// acceleration were exceded.
static inline inference_e detect_launch()
{
  if (stats[rock.buf].avg_vel >= LAUNCH_MIN_VEL &&
      stats[rock.buf].avg_vax >= LAUNCH_MIN_VAX)
  {
    rock.state = LAUNCH;
    rock.samp.ascent = 0;
    log_msg("DEPL: Launch detected", 22);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }

  return DEPL_OK;
}

/// Monitors height and velocity increase consistency.
static inline inference_e detect_ascent()
{
  if (stats[rock.buf].avg_vel > stats[!rock.buf].avg_vel &&
      stats[rock.buf].min_alt > stats[!rock.buf].max_alt)
  {
    ++rock.samp.ascent;
    if (rock.samp.ascent >= MIN_SAMP_ASCENT)
    {
      rock.state = ASCENT;
      rock.samp.burnout = 0;
      log_msg("DEPL: Launch confirmed", 23);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samp.ascent > 0)
  {
    rock.samp.ascent = 0;
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
  if (stats[rock.buf].avg_vel >= BURNOUT_MIN_VEL &&
      stats[rock.buf].avg_vax <= BURNOUT_MAX_VAX &&
      stats[rock.buf].max_alt > stats[rock.buf].min_alt &&
      stats[rock.buf].avg_vel < stats[!rock.buf].avg_vel)
  {
    ++rock.samp.burnout;
    if (rock.samp.burnout >= MIN_SAMP_BURNOUT)
    {
      rock.state = BURNOUT;
      log_msg("DEPL: Watching for apogee", 26);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samp.burnout > 0)
  {
    rock.samp.burnout = 0;
    return DEPL_N_BURNOUT;
  }
#endif

  return DEPL_OK;
}

/// Initially monitors for continuing burnout and
/// for velocity to pass the minimum threshold.
static inline inference_e detect_apogee()
{
  if (stats[rock.buf].avg_vel <= APOGEE_MAX_VEL &&
      stats[rock.buf].avg_vel < stats[!rock.buf].avg_vel)
  {
    rock.state = APOGEE;
    rock.samp.descent = 0;
    log_msg("DEPL: Approaching apogee", 25);
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }

  return DEPL_OK;
}

/// Monitors for decreasing altitude and increasing velocity.
static inline inference_e detect_descent()
{
  if (stats[rock.buf].max_alt < stats[!rock.buf].min_alt &&
      stats[rock.buf].avg_vel > stats[!rock.buf].avg_vel)
  {
    ++rock.samp.descent;
    if (rock.samp.descent >= MIN_SAMP_DESCENT)
    {
      rock.state = DESCENT;
      rock.samp.landing = 0;
      co2_high();
      log_msg("DEPL: Fired pyro, descending", 29);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samp.descent > 0)
  {
    rock.samp.descent = 0;
    return DEPL_N_DESCENT;
  }
#endif

  return DEPL_OK;
}

/// Monitors for falling below a specific altitude,
/// and checks for altitude consistency.
static inline inference_e detect_reef()
{
  if (stats[rock.buf].min_alt <= REEF_TARGET_ALT && 
      stats[rock.buf].max_alt < stats[!rock.buf].min_alt)
  {
    ++rock.samp.landing;
    if (rock.samp.landing >= MIN_SAMP_REEF)
    {
      rock.state = REEF;
      rock.samp.idle = 0;
      reef_low();
      log_msg("DEPL: Expanded parachute", 25);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samp.landing > 0)
  {
    rock.samp.landing = 0;
    return DEPL_N_REEF;
  }
#endif

  return DEPL_OK;
}

/// Monitors all statistical metrics to not deviate
/// beyond allowed tolerance thresholds.
static inline inference_e detect_landed()
{
  float dh = stats[rock.buf].min_alt - stats[!rock.buf].min_alt;
  float dv = stats[rock.buf].avg_vel - stats[!rock.buf].avg_vel;
  float da = stats[rock.buf].avg_vax - stats[!rock.buf].avg_vax;

  if ((dh <= ALT_TOLER && dh >= -ALT_TOLER) &&
      (dv <= VEL_TOLER && dv >= -VEL_TOLER) &&
      (da <= VAX_TOLER && da >= -VAX_TOLER))
  {
    ++rock.samp.idle;
    if (rock.samp.idle >= MIN_SAMP_LANDED)
    {
      rock.state = LANDED;
      log_msg("DEPL: Rocket landed", 20);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samp.idle > 0)
  {
    rock.samp.idle = 0;
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

  if (st == DEPL_OK) {
    /* Skip other branches */
  } else if (st == DATA_NONE) {
    rock.warn = st;
    return DEPL_OK;
  } else if (st > DEPL_OK) {
    rock.warn = st;
  } else if (st < DEPL_OK) {
    return st;
  }

  switch (rock.state) {
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


/* ------ Troubleshooting and recovery ------ */

/// Tries to reinitializes sensors and invalidate
/// data cache. Reassigns the rocket its active state.
static inline void try_recover_sensors()
{
  inference_e st = DEPL_OK;

  log_msg("DEPL: Trying to reinitialize sensors", 37);

  __disable_irq();

  if (init_baro() != HAL_OK)
    st += DEPL_BAD_BARO;

  if (init_gyro() != HAL_OK)
    st += DEPL_BAD_GYRO;

  if (init_accel() != HAL_OK)
    st += DEPL_BAD_ACCEL;

  invalidate_dcache(); /* non-IT (synchronous) */

  __enable_irq();
  
  if (st == DEPL_OK) {
    log_msg("DEPL: Sensor recovery successful", 33);
  } else {
    log_err("DEPL: Recovery failed (code %d)", st);
  }

  /* Given UKF time to process new data */
  tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP * 3);
}

/// Keeps record of successive retries,
/// and updates rocket state to Recovery or Aborted
/// if a corresponding retry threshold was exceeded.
///
/// Thresholds are configurable in deployment.h.
static inline void bad_inference_handler(inference_e inf)
{
  ++rock.retry;
  log_err("DEPL: bad inference #%u (code %d)", rock.retry, inf);

  if (rock.retry >= PRE_RECOV_RETRIES + PRE_ABORT_RETRIES)
  {
    rock.abort |= AUTO_ABORT;
    log_err("DEPL: aborting cycle (%u retries)", rock.retry);
  }
  else if (rock.retry >= PRE_RECOV_RETRIES &&
           rock.retry % RECOVERY_INTERVAL == 0)
  {
    try_recover_sensors();
  }
}

/// Give up inference attempts and wait for signals.
static inline void manual_mode()
{
  log_msg("DEPL: Entering manual mode", 27);

  while (SEDS_ARE_COOL) {
    if (rock.cmd == FIRE_PYRO &&
        !(rock.abort & PYRO_FIRED))
    {
      co2_high();
      rock.abort |= PYRO_FIRED;
    }
    else if (rock.cmd == FIRE_REEF &&
             rock.abort & PYRO_FIRED)
    {
      reef_high();
      return;
    }
    else if (rock.cmd == SHUTDOWN) return;

    tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP * 2);
  }
}

/* ------ Thread creation helper and main function ------ */

/// Invokes state machine, handles errors, retries,
/// and abortion. Resorts to manual mode.
///
/// Idle time is configurable in FC-Threads.h.
void deployment_thread_entry(ULONG cycle)
{
  if (cycle == DEPLOYMENT_THREAD_INPUT) {
    co2_low();
    reef_low();
    rock.state = IDLE;
  } else {
    rock.abort &= ~AUTO_ABORT;
    rock.warn = DEPL_OK;
    rock.retry = 0u;
  }

  while (!rock.abort) {
    inference_e inf = infer_rocket_state();

    if (inf == DATA_NONE) {
      tx_thread_resume(&ukf_thread);
      continue;
    } else if (inf < DEPL_OK) {
      bad_inference_handler(inf);
    } else if (rock.warn != DEPL_OK) {
      rock.warn = DEPL_OK;
      log_err("DEPL: non-critical warning code %d", inf);
    } else if (rock.retry > 0) {
      rock.retry = 0u;
      log_msg("DEPL: recovered from error", 27);
    }

    tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP);
  }

#if DEPLOYMENT_RESTART_ON_FAIL > 0
  if (rock.abort & MANUAL_ABORT || cycle >= MAX_CYCLES) {
    manual_mode();

    /* Directed by            Fuad
     * Consulting Producer    Rylan
     * Executive Producer     Parth
     * Copyright              UB SEDS 2026 */
    return;
  }

  /* Small stack outside of while loop => safe recursion */
  deployment_thread_entry(++cycle);

#else
  manual_mode();

#endif
}

/// Creates a non-preemptive deployment thread with
/// defined parameters. Called manually.
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

  if (st != TX_SUCCESS)
    log_die("Failed to create deployment thread: %u", (unsigned)st);
  else
    log_msg_sync("Starting deployment thread", 27);
}