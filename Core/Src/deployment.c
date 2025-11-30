/*
 * Logic related to state monitoring and parachute deployment.
 */

#include "deployment.h"
#include <stdint.h>

TX_THREAD deployment_thread;
ULONG deployment_thread_stack[DEPLOYMENT_THREAD_STACK_SIZE / sizeof(ULONG)];

static rocket_t rock = {0, {0}, {0, 0, 0, 0}};
static filter_t data[2][DEPL_BUF_SIZE] = {{0}, {0}};
static stats_t stats[2] = {0};
extern atomic_uint_fast16_t newdata;

/*
 * Copies "current" data into "previous" buffer and
 * updates "current" with new data from the filter ring.
 *
 * rock.i.klm - index of last copied ring entry,
 * rock.i.cur - current "size" of "current" data,
 * rock.i.old - current "size" of "previous" data.
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
  uint_fast16_t n;
  uint_fast16_t d = 0;
  uint_fast16_t k = rock.i.klm;

  if (!(n = atomic_exchange_explicit(&newdata, 0, memory_order_acq_rel)))
  {
    // tx_thread_resume(&kalman_thread);
    return DEPL_NO_INPUT;
  }
  else if (n > DEPL_BUF_SIZE)
  {
    // k = (k + n - DEPL_BUF_SIZE) & (KALMAN_RING_SIZE - 1);
  }

  rock.i.r = !rock.i.r;

  for (; d < MIN(n, DEPL_BUF_SIZE); ++d)
  {
    // k = (k + 1) & (KALMAN_RING_SIZE - 1);
    // lock kalman_ring[k]
    // data[rock.i.last][d] = kalman_ring[rock.i.klm];
    // unlock kalman_ring[k]
  }

  rock.i.klm = k;
  rock.i.prv = rock.i.cur;
  rock.i.cur = d;

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
    if (data[rock.i.r][j].height_m    < MIN_HEIGHT_M    ||
        data[rock.i.r][j].height_m    > MAX_HEIGHT_M)
    {
      st += DEPL_BAD_HEIGHT;
    }
    if (data[rock.i.r][j].vel_mps     < MIN_VEL_MPS     ||
        data[rock.i.r][j].vel_mps     > MAX_VEL_MPS)
    {
      st += DEPL_BAD_VEL;
    }
    if (data[rock.i.r][j].accel_mps2  < MIN_ACCEL_MPS2  ||
        data[rock.i.r][j].accel_mps2  > MAX_ACCEL_MPS2)
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
  float sum_vel = data[rock.i.r][0].vel_mps;

  stats[rock.i.r].max_height_m   = data[rock.i.r][0].height_m;
  stats[rock.i.r].min_height_m   = data[rock.i.r][0].height_m;
  stats[rock.i.r].min_accel_mps2 = data[rock.i.r][0].accel_mps2;

  for (uint_fast8_t j = 1; j < rock.i.cur; ++j)
  {
    float h = data[rock.i.r][j].height_m;
    float a = data[rock.i.r][j].accel_mps2;

    if (h < stats[rock.i.r].min_height_m)
    {
      stats[rock.i.r].min_height_m = h;
    }
    else if (h > stats[rock.i.r].max_height_m)
    {
      stats[rock.i.r].max_height_m = h;
    }

    if (a < stats[rock.i.r].min_accel_mps2)
    {
      stats[rock.i.r].min_accel_mps2 = a;
    }

    sum_vel += data[rock.i.r][j].vel_mps;
  }

  stats[rock.i.r].mean_vel_mps = sum_vel / (float)rock.i.cur;
}

/*
 * Monitors if minimum thresholds for velocity and
 * acceleration were exceded. Checks for height and
 * velocity increase consistancy. Revertable.
 */
static inline inference_e detect_launch(inference_e mode)
{
  if (stats[rock.i.r].mean_vel_mps >= LAUNCH_MIN_MEAN_VEL_MPS &&
      stats[rock.i.r].min_accel_mps2 >= LAUNCH_MIN_ACCEL_MPS2 &&

      stats[rock.i.r].mean_vel_mps > stats[!rock.i.r].mean_vel_mps &&
      stats[rock.i.r].min_height_m > stats[!rock.i.r].max_height_m)
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
      rock.samp_of.burnout = 0;
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
  if (stats[rock.i.r].mean_vel_mps >= BURNOUT_MIN_MEAN_VEL_MPS &&
      stats[rock.i.r].min_accel_mps2 <= BURNOUT_MAX_ACCEL_MPS2 &&

      stats[rock.i.r].max_height_m > stats[rock.i.r].min_height_m &&
      stats[rock.i.r].mean_vel_mps < stats[!rock.i.r].mean_vel_mps)
  {
    ++rock.samp_of.burnout;
    if (rock.samp_of.burnout >= MIN_SAMP_BURNOUT)
    {
      rock.state = BURNOUT;
      rock.samp_of.descent = 0;
      LOG_MSG("Watching for apogee", 20);
    }
  }
  else if (rock.samp_of.burnout > 0)
  {
    rock.samp_of.burnout = 0;
    return DEPL_N_BURNOUT;
  }

  return DEPL_OK;
}

static inline inference_e detect_apogee(inference_e mode)
{
  rock.state = APOGEE;
  LOG_MSG("Apogee reached", 15);
  WAIT_BEFORE_CONFIRM();

  ++rock.samp_of.descent;
  if (rock.samp_of.descent >= MIN_SAMP_DESCENT)
  {
    rock.state = DESCENT;
    rock.samp_of.landing = 0;
    CO2_HIGH();
    LOG_MSG("Fired pyro, descending", 23);
  }

  return DEPL_OK;
}

static inline inference_e detect_reef()
{
  ++rock.samp_of.landing;
  if (rock.samp_of.landing >= MIN_SAMP_REEF)
  {
    rock.state = REEF;
    rock.samp_of.idle = 0;
    REEF_HIGH();
    LOG_MSG("Expanded parachute", 19);
  }
  return DEPL_OK;
}

static inline inference_e detect_landed()
{
  ++rock.samp_of.idle;
  if (rock.samp_of.idle >= MIN_SAMP_LANDED)
  {
    rock.state = LANDED;
    LOG_MSG("Rocket landed", 14);
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

  if ((st = refresh_data()) == DEPL_NO_INPUT)
    return st;

  if ((st = verify_data()) < DEPL_OK)
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