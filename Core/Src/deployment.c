/*
 * State monitoring and parachute deployment.
 * Designed to run as a task in one thread. 
 */

#include "deployment.h"

TX_THREAD deployment_thread;
ULONG deployment_thread_stack[DEPLOYMENT_THREAD_STACK_SIZE / sizeof(ULONG)];

static rocket_t rock = {0, {0}, {0, 0, 0, 0}};
static filter_t data[2][DEPL_BUF_SIZE] = {{0}, {0}};
static stats_t stats[2] = {0};
extern atomic_uint_fast8_t newdata;

/*
 * Copies "current" data into "previous" buffer and
 * updates "current" with new data from the filter ring.
 *
 * rock.i.ext - index of last copied ring entry,
 * rock.i.sc  - current "size" of "current" data,
 * rock.i.sp  - current "size" of "previous" data.
 *
 * newdata (n) - external atomic counter for new elements,
 * should be incremented each time filter writes to ring.
 *
 * We compute starting index (k) in the ring by adding 1
 * (modulo ring size) to the index of last copied entry.
 * If newdata exceeds our local buffer size, we advance
 * our starting position to copy the latest of new entries.
 *
 * Every iteration we lock the active element of the ring
 * to avoid partial reads, and to let other entries update.
 * 
 * If the deployment task is switched to filter task while
 * the second loop was running, newdata will grow. If the
 * filter encounters our lock, it should starve or yield.
 *
 * When this task is back, it will still use its old copy
 * of newdata, and stop exactly before the first new element.
 * When this function is called next time, it will begin
 * copying new data, advancing its starting index if needed.
 */
static inline inference_e refresh_data()
{
  uint_fast8_t n;

  if (!(n = atomic_exchange_explicit(&newdata, 0, memory_order_acq_rel)))
  {
    // tx_thread_resume(&kalman_thread);
    return DEPL_NO_INPUT;
  }
  else if (n > DEPL_BUF_SIZE)
  {
    // k = (k + n - DEPL_BUF_SIZE) & (KALMAN_RING_SIZE - 1);
  }

  rock.i.a = !rock.i.a;
  rock.i.sp = rock.i.sc;

  for (rock.i.sc = 0; rock.i.sc < MIN(n, DEPL_BUF_SIZE); ++rock.i.sc)
  {
    // rock.i.ex = (rock.i.ex + 1) & (KALMAN_RING_SIZE - 1);
    // lock kalman_ring[rock.i.ex]
    // data[rock.i.last][d] = kalman_ring[rock.i.ex];
    // unlock kalman_ring[rock.i.ex]
  }

  return DEPL_OK;
}

/*
 * Checks if every metric is within allowed bounds.
 * 
 * If this function fails enough times, it will abort
 * the deployment thread. Set thresholds carefully.
 */
static inline inference_e verify_data()
{
  inference_e st = DEPL_OK;

  for (uint_fast8_t j = 0; j < DEPL_BUF_SIZE; ++j)
  {
    if (data[rock.i.a][j].alt < SANITY_MIN_ALT ||
        data[rock.i.a][j].alt > SANITY_MAX_ALT)
    {
      st += DEPL_BAD_HEIGHT;
    }
    if (data[rock.i.a][j].vel < SANITY_MIN_VEL ||
        data[rock.i.a][j].vel > SANITY_MAX_VEL)
    {
      st += DEPL_BAD_VEL;
    }
    if (data[rock.i.a][j].vax < SANITY_MIN_VAX ||
        data[rock.i.a][j].vax > SANITY_MAX_VAX)
    {
      st += DEPL_BAD_ACCEL;
    }
  }

  return st;
}

/*
 * Updates instantaneous flight statistics based
 * on data from the "current" buffer.
 *
 * Currently updates lowest and largest height,
 * average velocity, and lowest acceleration reported.
 * Other metrics can be added later if necessary.
 *
 * refresh_data() guarantees that if this function
 * is called, it does not receive old or empty data.
 */
static inline void refresh_stats()
{
  float sum_vel = data[rock.i.a][0].vel;
  float sum_vax = data[rock.i.a][0].vax;

  stats[rock.i.a].max_alt = data[rock.i.a][0].alt;
  stats[rock.i.a].min_alt = data[rock.i.a][0].alt;

  for (uint_fast8_t j = 1; j < rock.i.sc; ++j)
  {
    stats[rock.i.a].min_alt = MIN(stats[rock.i.a].min_alt,
                                  data[rock.i.a][j].alt);

    stats[rock.i.a].max_alt = MAX(stats[rock.i.a].max_alt,
                                  data[rock.i.a][j].alt);

    sum_vel += data[rock.i.a][j].vel;
    sum_vax += data[rock.i.a][j].vax;
  }

  stats[rock.i.a].avg_vel = sum_vel / (float)rock.i.sc;
  stats[rock.i.a].avg_vax = sum_vax / (float)rock.i.sc;
}

/*
 * Monitors if minimum thresholds for velocity and
 * acceleration were exceded. Checks for height and
 * velocity increase consistancy. Revertable.
 */
static inline inference_e detect_launch(inference_e mode)
{
  if (stats[rock.i.a].avg_vel >= LAUNCH_MIN_VEL &&
      stats[rock.i.a].avg_vax >= LAUNCH_MIN_VAX &&
      stats[rock.i.a].avg_vel > stats[!rock.i.a].avg_vel &&
      stats[rock.i.a].min_alt > stats[!rock.i.a].max_alt)
  {
    if (mode == INFER_INITIAL)
    {
      rock.state = LAUNCH;
      LOG_MSG("Launch detected", 16);
      WAIT_BEFORE_CONFIRM();
    }
    else if (mode == INFER_CONFIRM)
    {
      rock.state = ASCENT;
      rock.samp.burnout = 0;
      LOG_MSG("Launch confirmed", 17);
    }
  }
  else if (mode == INFER_CONFIRM)
  {
    rock.state = IDLE;
    return DEPL_F_LAUNCH;
  }

  return DEPL_OK;
}

/*
 * Monitors if minimum threshold for velocity and
 * maximum threshold for acceleration were passed.
 * Checks for height increase and velocity decrease
 * consistency. Non-revertable.
 */
static inline inference_e detect_burnout()
{
  if (stats[rock.i.a].avg_vel >= BURNOUT_MIN_VEL &&
      stats[rock.i.a].avg_vax <= BURNOUT_MAX_VAX &&
      stats[rock.i.a].max_alt > stats[rock.i.a].min_alt &&
      stats[rock.i.a].avg_vel < stats[!rock.i.a].avg_vel)
  {
    ++rock.samp.burnout;
    if (rock.samp.burnout >= MIN_SAMP_BURNOUT)
    {
      rock.state = BURNOUT;
      rock.samp.descent = 0;
      LOG_MSG("Watching for apogee", 20);
    }
  }
  else if (rock.samp.burnout > 0)
  {
    rock.samp.burnout = 0;
    return DEPL_N_BURNOUT;
  }

  return DEPL_OK;
}

/*
 * Initially monitors for decreasing velocity, then waits
 * and begins to monitor for decreasing altirude and
 * increasing velocity. Non-revertable.
 */
static inline inference_e detect_apogee(inference_e mode)
{
  if (stats[rock.i.a].avg_vel <= APOGEE_MAX_VEL &&
      stats[rock.i.a].avg_vel < stats[!rock.i.a].avg_vel)
  {
    if (mode == INFER_INITIAL)
    {
      rock.state = APOGEE;
      LOG_MSG("Approaching apogee", 19);
      WAIT_BEFORE_CONFIRM();
    }
    else if (mode == INFER_CONFIRM)
    {
      return DEPL_F_APOGEE;
    }
  }
  else if (mode == INFER_CONFIRM)
  {
    if (stats[rock.i.a].max_alt < stats[!rock.i.a].min_alt &&
        stats[rock.i.a].avg_vel > stats[!rock.i.a].avg_vel)
    {
      ++rock.samp.descent;
      if (rock.samp.descent >= MIN_SAMP_DESCENT)
      {
        rock.state = DESCENT;
        rock.samp.landing = 0;
        CO2_HIGH();
        LOG_MSG("Fired pyro, descending", 23);
      }
    }
    else if (rock.samp.descent > 0)
    {
      rock.samp.descent = 0;
      return DEPL_N_DESCENT;
    }
  }

  return DEPL_OK;
}

/*
 * Monitors for falling below a specific altitude,
 * and checks for altitude consistency. Non-revertable.
 */
static inline inference_e detect_reef()
{
  if (stats[rock.i.a].min_alt <= REEF_TARGET_ALT && 
      stats[rock.i.a].max_alt < stats[!rock.i.a].min_alt)
  {
    ++rock.samp.landing;
    if (rock.samp.landing >= MIN_SAMP_REEF)
    {
      rock.state = REEF;
      rock.samp.idle = 0;
      REEF_HIGH();
      LOG_MSG("Expanded parachute", 19);
    }
  }
  else if (rock.samp.landing > 0)
  {
    rock.samp.landing = 0;
    return DEPL_N_REEF;
  }

  return DEPL_OK;
}

/*
 * Monitors all statistical metrics to not deviate
 * beyond allowed tolerance thresholds. Non-revertable.
 */
static inline inference_e detect_landed()
{
  if (stats[rock.i.a].min_alt - stats[!rock.i.a].min_alt <= ALT_TOLER &&
      stats[rock.i.a].avg_vel - stats[!rock.i.a].avg_vel <= VEL_TOLER &&
      stats[rock.i.a].avg_vax - stats[!rock.i.a].avg_vax <= VAX_TOLER)
  {
    ++rock.samp.idle;
    if (rock.samp.idle >= MIN_SAMP_LANDED)
    {
      rock.state = LANDED;
      LOG_MSG("Rocket landed", 14);
    }
  }
  else if (rock.samp.idle > 0)
  {
    rock.samp.idle = 0;
    return DEPL_N_LANDED;
  }

  return DEPL_OK;
}

/*
 * This is a state machine that calls functions necessary
 * to gather, validate, process, and draw inference from data.
 *
 * Current implementation does not allow state regression
 * between different functions (but allows within one function),
 * and triggers checks to prevent premature or wrong transitions.
 *
 * Amount of checks and thresholds are configurable in deployment.h.
 */
static inline inference_e infer_rocket_state()
{
  inference_e st;

  if ((st = refresh_data()) != DEPL_OK)
    return st;

  if ((st = verify_data()) != DEPL_OK)
    return st;

  refresh_stats();

  switch (rock.state)
  {
    case IDLE:
      return detect_launch(INFER_INITIAL);
    case LAUNCH:
      return detect_launch(INFER_CONFIRM);
    case ASCENT:
      return detect_burnout();
    case BURNOUT:
      return detect_apogee(INFER_INITIAL);
    case APOGEE:
      return detect_apogee(INFER_CONFIRM);
    case DESCENT:
      return detect_reef();
    case REEF:
      return detect_landed();
    case LANDED:
      return DEPL_OK;
  }
}

/*
 * Entry point of the deployment thread. Usually called by RTOS.
 *
 * Returns (aborts thread) if the state machine returned an error
 * certain amount of times in a row (that is, if the error could
 * not be mitigated within a given time).
 *
 * Amount of retries and idle time are configurable in FC-Threads.h.
 */
void deployment_thread_entry(ULONG input)
{
  (void)input;
  uint_fast16_t retries = 0;
  inference_e last = DEPL_OK;

  LOG_MSG_SYNC("Deployment: starting thread", 28);

  CO2_LOW();
  REEF_LOW();

  for (;;)
  {
    if ((last = infer_rocket_state()) == DEPL_OK)
    {
      retries = 0; // Reset counter (error mitigated)
    }
    else if (last > DEPL_OK)
    {
      LOG_ERR("Deployment: non-critical debug warning (code %d)", last);
    }
    else
    {
      LOG_ERR("Deployment: bad inference #%u (code %d)", retries, last);
      ++retries;

      if (retries >= DEPLOYMENT_THREAD_MAX_RETRIES)
      {
        LOG_ERR_SYNC("FATAL: aborting deployment (%u retries)", retries);
        return;
      }
    }
    tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP);
  }
}

/*
 * Creates a non-preemptive deployment thread with defined parameters.
 * Called manually. Provides its entry point with a throwaway input.
 *
 * Stack size and priority are configurable in FC-Threads.h.
 */
void create_deployment_thread(void)
{
  UINT st = tx_thread_create(&deployment_thread,
                             "Deployment Thread",
                             deployment_thread_entry,
                             DEPLOYMENT_THREAD_INPUT,
                             deployment_thread_stack,
                             DEPLOYMENT_THREAD_STACK_SIZE,
                             DEPLOYMENT_THREAD_PRIORITY,
                             DEPLOYMENT_THREAD_PRIORITY, // No preemption
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS)
  {
    die("Failed to create deployment thread: %u", (unsigned)st);
  }
}