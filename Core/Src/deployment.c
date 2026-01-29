/*
 * State monitoring and parachute deployment.
 * Designed to run as a task *in one thread*.
 */

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
static monitor_t monitor = {0};

/// Generalized UKF metrics (double buffering).
static struct stats stats[2] = {0};


/* ------ Public API ------ */

/// Does not cover states before rocket is ready to launch.
enum state current_flight_state() { return rock.state; }

/// Send enum-specified command to deployment.
/// First command should be 'ABORT' to enter manual mode.
void deployment_send_command(command_e command)
{
  if (command == ABORT) {
    monitor.abort |= MANUAL_ABORT;
  } else {
    monitor.cmd = command;
  }
}


/* ------ Data acquisition ------ */

/// Helper to determine max/min values on an interval.
static inline void acc_abs(fetch_t *ft, float val, max, min,
                           uint_fast8_t ok, inference_e err,
                           float *abs_min, float *abs_max)
{
  if (val < min || val > max) {
    ft->st += err;
  } else if (ft->mask & ok) {
    *abs_min = MIN(val, *abs_min);
    *abs_max = MAX(val, *abs_max);
  } else {
    *abs_min = val;
    *abs_max = val;
    ft->mask |= ok;
  }
}

/// Helper to calculate sum (average) on an interval. 
static inline void acc_avg(fetch_t *ft, float val, max, min,
                           uint_fast8_t ok, inference_e err,
                           uint_fast8_t *count, float *sum)
{
  if (val < min || val > max) {
    ft->st += err;
  } else if (ft->mask & ok) {
    *sum += val;
    ++(*count);
  } else {
    *sum = val;
    ft->mask |= ok;
  }
}

/// Checks 1 to 4 new values in ring against sanity boundaries,
/// and averages valid values as defined in struct stats (deployment.h).
/// Partially valid samples are supported.
///
/// Returns DATA_NONE if ring has no new values, > 0 if some data
/// was rejected but new struct stats was formed, and < 0 if provided
/// valid data was not enough to form a complete struct stats;
static inline inference_e refresh_stats()
{
  static filter_t data[DATA_CAP];

  uint_fast8_t k = ukf_fetch(data);
  if (!k)
    return DATA_NONE;

  rock.a = !rock.a;
  fetch_t ft = { DATA_OFFSET, 0 };
  uint_fast8_t vel = 1u, vax = 1u;
  float sum_vel = 0.0f, sum_vax = 0.0f;

  for (; k > 0; --k)
  {
    acc_abs(&ft, data[k].alt, MAX_ALT, MIN_ALT, VALID_ALT,
            DATA_BAD_ALT, &stats[rock.a].min_alt,
            &stats[rock.a].max_alt);

    acc_avg(&ft, data[k].vel, MAX_VEL, MIN_VEL,
            VALID_VEL, DATA_BAD_VEL, &vel, &sum_vel);

    acc_avg(&ft, data[k].vax, MAX_VAX, MIN_VAX,
            VALID_VAX, DATA_BAD_VAX, &vax, &sum_vax);
  }

  if ((ft.mask & VALID_STATS) != VALID_STATS) {
    /* Write to the same buffer next time. */
    rock.a = !rock.a;
    return ft.st;
  }

  stats[rock.a].avg_vel = sum_vel / (float)vel;
  stats[rock.a].avg_vax = sum_vax / (float)vax;

  return (ft.st == DATA_OFFSET) ? DEPL_OK : -ft.st;
}


/* ------ State machine ------ */

/// Monitors if minimum thresholds for velocity and
/// acceleration were exceded.
static inline inference_e detect_launch()
{
  if (stats[rock.a].avg_vel >= LAUNCH_MIN_VEL &&
      stats[rock.a].avg_vax >= LAUNCH_MIN_VAX)
  {
    rock.state = LAUNCH;
    log_msg("DEPL: Launch detected", 22);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }

  return DEPL_OK;
}

/// Monitors height and velocity increase consistency.
static inline inference_e detect_ascent()
{
  if (stats[rock.a].avg_vel > stats[!rock.a].avg_vel &&
      stats[rock.a].min_alt > stats[!rock.a].max_alt)
  {
    ++rock.samples;
    if (rock.samples >= MIN_SAMP_ASCENT)
    {
      rock.state = ASCENT;
      rock.samples = 0;
      log_msg("DEPL: Launch confirmed", 23);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samples > 0)
  {
    rock.samples = 0;
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
  if (stats[rock.a].avg_vel >= BURNOUT_MIN_VEL &&
      stats[rock.a].avg_vax <= BURNOUT_MAX_VAX &&
      stats[rock.a].max_alt > stats[rock.a].min_alt &&
      stats[rock.a].avg_vel < stats[!rock.a].avg_vel)
  {
    ++rock.samples;
    if (rock.samples >= MIN_SAMP_BURNOUT)
    {
      rock.state = BURNOUT;
      log_msg("DEPL: Watching for apogee", 26);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samples > 0)
  {
    rock.samples = 0;
    return DEPL_N_BURNOUT;
  }
#endif

  return DEPL_OK;
}

/// Initially monitors for continuing burnout and
/// for velocity to pass the minimum threshold.
static inline inference_e detect_apogee()
{
  if (stats[rock.a].avg_vel <= APOGEE_MAX_VEL &&
      stats[rock.a].avg_vel < stats[!rock.a].avg_vel)
  {
    rock.state = APOGEE;
    rock.samples = 0;
    log_msg("DEPL: Approaching apogee", 25);
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }

  return DEPL_OK;
}

/// Monitors for decreasing altitude and increasing velocity.
static inline inference_e detect_descent()
{
  if (stats[rock.a].max_alt < stats[!rock.a].min_alt &&
      stats[rock.a].avg_vel > stats[!rock.a].avg_vel)
  {
    ++rock.samples;
    if (rock.samples >= MIN_SAMP_DESCENT)
    {
      rock.state = DESCENT;
      rock.samples = 0;
      co2_high();
      log_msg("DEPL: Fired pyro, descending", 29);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samples > 0)
  {
    rock.samples = 0;
    return DEPL_N_DESCENT;
  }
#endif

  return DEPL_OK;
}

/// Monitors for falling below a specific altitude,
/// and checks for altitude consistency.
static inline inference_e detect_reef()
{
  if (stats[rock.a].min_alt <= REEF_TARGET_ALT && 
      stats[rock.a].max_alt < stats[!rock.a].min_alt)
  {
    ++rock.samples;
    if (rock.samples>= MIN_SAMP_REEF)
    {
      rock.state = REEF;
      rock.samples = 0;
      reef_low();
      log_msg("DEPL: Expanded parachute", 25);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samples > 0)
  {
    rock.samples = 0;
    return DEPL_N_REEF;
  }
#endif

  return DEPL_OK;
}

/// Monitors all statistical metrics to not deviate
/// beyond allowed tolerance thresholds.
static inline inference_e detect_landed()
{
  float dh = stats[rock.a].min_alt - stats[!rock.a].min_alt;
  float dv = stats[rock.a].avg_vel - stats[!rock.a].avg_vel;
  float da = stats[rock.a].avg_vax - stats[!rock.a].avg_vax;

  if ((dh <= ALT_TOLER && dh >= -ALT_TOLER) &&
      (dv <= VEL_TOLER && dv >= -VEL_TOLER) &&
      (da <= VAX_TOLER && da >= -VAX_TOLER))
  {
    ++rock.samples;
    if (rock.samples >= MIN_SAMP_LANDED)
    {
      rock.state = LANDED;
      log_msg("DEPL: Rocket landed", 20);
    }
  }
#if defined CONSECUTIVE_CONFIRMS
  else if (rock.samples > 0)
  {
    rock.samples = 0;
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
    monitor.warn = st;
    return DEPL_OK;
  } else if (st > DEPL_OK) {
    monitor.warn = st;
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
  ++monitor.retry;
  log_err("DEPL: bad inference #%u (code %d)", monitor.retry, inf);

  if (monitor.retry >= PRE_RECOV_RETRIES + PRE_ABORT_RETRIES)
  {
    monitor.abort |= AUTO_ABORT;
    log_err("DEPL: aborting cycle (%u retries)", monitor.retry);
  }
  else if (monitor.retry >= PRE_RECOV_RETRIES &&
           monitor.retry % RECOVERY_INTERVAL == 0)
  {
    try_recover_sensors();
  }
}

/// Give up inference attempts and wait for signals.
static inline void manual_mode()
{
  log_msg("DEPL: Entering manual mode", 27);

  while (SEDS_ARE_COOL) {
    if (monitor.cmd == FIRE_PYRO &&
        !(monitor.abort & PYRO_FIRED))
    {
      co2_high();
      monitor.abort |= PYRO_FIRED;
    }
    else if (monitor.cmd == FIRE_REEF &&
             monitor.abort & PYRO_FIRED)
    {
      reef_high();
      return;
    }
    else if (monitor.cmd == SHUTDOWN) return;

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
    monitor.abort &= ~AUTO_ABORT;
    monitor.warn = DEPL_OK;
    monitor.retry = 0u;
  }

  while (!monitor.abort) {
    inference_e inf = infer_rocket_state();

    if (inf == DATA_NONE) {
      tx_thread_resume(&ukf_thread);
      continue;
    } else if (inf < DEPL_OK) {
      bad_inference_handler(inf);
    } else if (monitor.warn != DEPL_OK) {
      monitor.warn = DEPL_OK;
      log_err("DEPL: non-critical warning code %d", inf);
    } else if (monitor.retry > 0) {
      monitor.retry = 0u;
      log_msg("DEPL: recovered from error", 27);
    }

    tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP);
  }

#if DEPLOYMENT_RESTART_ON_FAIL > 0
  if (monitor.abort & MANUAL_ABORT || cycle >= MAX_CYCLES) {
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

  if (st != TX_SUCCESS) {
    log_die("Failed to create deployment thread: %u", (unsigned)st);
  } else {
    log_msg_sync("Starting deployment thread", 27);
  }
}