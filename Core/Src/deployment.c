/*
 * State monitoring and parachute deployment.
 * Designed to run as a task *in one thread*. 
 */

#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include "gyro.h"
//#include "accel.h"
#include "barometer.h"

#include "deployment.h"
#include <sedsprintf.h>
#include "telemetry.h"
#include "FC-Threads.h"

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_def.h"
#include "stm32h5xx_hal_spi.h"
#include "stm32h5xx_hal_gpio.h"
#include "stm32h5xx_hal_dcache.h"

TX_THREAD deployment_thread;
ULONG deployment_thread_stack[DEPLOYMENT_THREAD_STACK_SIZE / sizeof(ULONG)];

static rocket_t rock = {0, {0}, {0, 0, 0, 0}, {0, 0, 0}};

/*
 * Most-recent and previous data buffers merged
 */
static filter_t data[2][DEPL_BUF_SIZE] = {{0}, {0}};

/*
 * External atomic counter for new elements 
 * Increment each time a new element is written to filter ring
 */
static stats_t stats[2] = {{0}, {0}};

extern SPI_HandleTypeDef hspi1;
extern DCACHE_HandleTypeDef hdcache1;
extern atomic_uint_fast8_t newdata;

/*
 * Copies "current" data into "previous" buffer and
 * updates "current" with new data from the filter ring.
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
static inline inference_e validate_data()
{
  inference_e st = DEPL_OK;

  for (uint_fast8_t j = 0; j < rock.i.sc; ++j)
  {
    if (data[rock.i.a][j].alt < SANITY_MIN_ALT ||
        data[rock.i.a][j].alt > SANITY_MAX_ALT)
    {
      st += DEPL_BAD_ALT;
    }
    if (data[rock.i.a][j].vel < SANITY_MIN_VEL ||
        data[rock.i.a][j].vel > SANITY_MAX_VEL)
    {
      st += DEPL_BAD_VEL;
    }
    if (data[rock.i.a][j].vax < SANITY_MIN_VAX ||
        data[rock.i.a][j].vax > SANITY_MAX_VAX)
    {
      st += DEPL_BAD_VAX;
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
 * acceleration were exceded.
 */
static inline inference_e detect_launch()
{
  if (stats[rock.i.a].avg_vel >= LAUNCH_MIN_VEL &&
      stats[rock.i.a].avg_vax >= LAUNCH_MIN_VAX)
  {
    rock.state = LAUNCH;
    rock.samp.ascent = 0;
    LOG_MSG("Launch detected", 16);
    WAIT_CONFIRM_LAUNCH();
  }

  return DEPL_OK;
}

/*
 * Monitors height and velocity increase consistency.
 */
static inline inference_e detect_ascent()
{
  if (stats[rock.i.a].avg_vel > stats[!rock.i.a].avg_vel &&
      stats[rock.i.a].min_alt > stats[!rock.i.a].max_alt)
  {
    ++rock.samp.ascent;
    if (rock.samp.ascent >= MIN_SAMP_ASCENT) {
      rock.state = ASCENT;
      rock.samp.burnout = 0;
      LOG_MSG("Launch confirmed", 17);
    }
  }
  else if (rock.samp.ascent > 0)
  {
    #if (CONSECUTIVE_CONFIRMS == 1)
    rock.samp.ascent = 0;
    #endif

    return DEPL_N_LAUNCH;
  }

  return DEPL_OK;
}

/*
 * Monitors if minimum threshold for velocity and
 * maximum threshold for acceleration were passed.
 * Checks for height increase and velocity decrease
 * consistency.
 */
static inline inference_e detect_burnout()
{
  if (stats[rock.i.a].avg_vel >= BURNOUT_MIN_VEL &&
      stats[rock.i.a].avg_vax <= BURNOUT_MAX_VAX &&
      stats[rock.i.a].max_alt > stats[rock.i.a].min_alt &&
      stats[rock.i.a].avg_vel < stats[!rock.i.a].avg_vel)
  {
    ++rock.samp.burnout;
    if (rock.samp.burnout >= MIN_SAMP_BURNOUT) {
      rock.state = BURNOUT;
      rock.samp.descent = 0;
      LOG_MSG("Watching for apogee", 20);
    }
  }
  else if (rock.samp.burnout > 0)
  {
    #if (CONSECUTIVE_CONFIRMS == 1)
    rock.samp.burnout = 0;
    #endif

    return DEPL_N_BURNOUT;
  }

  return DEPL_OK;
}

/*
 * Initially monitors for continuing burnout and
 * for velocity to pass the minimum threshold.
 */
static inline inference_e detect_apogee()
{
  if (stats[rock.i.a].avg_vel <= APOGEE_MAX_VEL &&
      stats[rock.i.a].avg_vel < stats[!rock.i.a].avg_vel)
  {
    rock.state = APOGEE;
    rock.samp.descent = 0;
    LOG_MSG("Approaching apogee", 19);
    WAIT_CONFIRM_APOGEE();
  }

  return DEPL_OK;
}

/*
 * Monitors for decreasing altitude and increasing velocity.
 */
static inline inference_e detect_descent()
{
  if (stats[rock.i.a].max_alt < stats[!rock.i.a].min_alt &&
      stats[rock.i.a].avg_vel > stats[!rock.i.a].avg_vel)
  {
    ++rock.samp.descent;
    if (rock.samp.descent >= MIN_SAMP_DESCENT) {
      rock.state = DESCENT;
      rock.samp.landing = 0;
      CO2_HIGH();
      LOG_MSG("Fired pyro, descending", 23);
    }
  }
  else if (rock.samp.descent > 0)
  {
    #if (CONSECUTIVE_CONFIRMS == 1)
    rock.samp.descent = 0;
    #endif

    return DEPL_N_DESCENT;
  }

  return DEPL_OK;
}

/*
 * Monitors for falling below a specific altitude,
 * and checks for altitude consistency.
 */
static inline inference_e detect_reef()
{
  if (stats[rock.i.a].min_alt <= REEF_TARGET_ALT && 
      stats[rock.i.a].max_alt < stats[!rock.i.a].min_alt)
  {
    ++rock.samp.landing;
    if (rock.samp.landing >= MIN_SAMP_REEF) {
      rock.state = REEF;
      rock.samp.idle = 0;
      REEF_HIGH();
      LOG_MSG("Expanded parachute", 19);
    }
  }
  else if (rock.samp.landing > 0)
  {
    #if (CONSECUTIVE_CONFIRMS == 1)
    rock.samp.landing = 0;
    #endif

    return DEPL_N_REEF;
  }

  return DEPL_OK;
}

/*
 * Monitors all statistical metrics to not deviate
 * beyond allowed tolerance thresholds.
 */
static inline inference_e detect_landed()
{
  float dh = stats[rock.i.a].min_alt - stats[!rock.i.a].min_alt;
  float dv = stats[rock.i.a].avg_vel - stats[!rock.i.a].avg_vel;
  float da = stats[rock.i.a].avg_vax - stats[!rock.i.a].avg_vax;

  if ((dh <= ALT_TOLER && dh >= -ALT_TOLER) &&
      (dv <= VEL_TOLER && dv >= -VEL_TOLER) &&
      (da <= VAX_TOLER && da >= -VAX_TOLER))
  {
    ++rock.samp.idle;
    if (rock.samp.idle >= MIN_SAMP_LANDED) {
      rock.state = LANDED;
      LOG_MSG("Rocket landed", 14);
    }
  }
  else if (rock.samp.idle > 0)
  {
    #if (CONSECUTIVE_CONFIRMS == 1)
    rock.samp.idle = 0;
    #endif

    return DEPL_N_LANDED;
  }

  return DEPL_OK;
}

/*
 * Currently this state machine does not allow state regression
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

  if ((st = validate_data()) != DEPL_OK)
    return st;

  refresh_stats();

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

    /* Assert unreachable */
    case RECOVERY:
    case ABORTED:
    default: return DEPL_DOOM;
  }
}

/*
 * Tries to reinitializes sensors and invalidate
 * data cache. Reassigns the rocket its active state.
 */
static inline void try_recover_sensors()
{
  inference_e st = DEPL_OK;

  LOG_MSG("Trying to recover sensors", 26);

  (void)__disable_irq;

  if (init_barometer(&hspi1) != HAL_OK)
    st += DEPL_BAD_BARO;

  if (gyro_init(&hspi1) != HAL_OK)
    st += DEPL_BAD_GYRO;

  // if (accel_init(&hspi1) != HAL_OK)
  //   st += DEPL_BAD_ACCEL;

  HAL_DCACHE_Invalidate(&hdcache1);

  (void)__enable_irq;
  
  if (st == DEPL_OK) {
    LOG_MSG("Sensor recovery successful", 27);
  } else {
    LOG_ERR("Recovery failed (code %d)", st);
  }

  /* Try to validate data anyway */
  rock.state = rock.rec.state;
}

/*
 * Keeps record of successive retries,
 * and updates rocket state to Recovery or Aborted
 * if a corresponding retry threshold was exceeded.
 *
 * Thresholds are configurable in deployment.h.
 */
static inline void bad_inference_handler()
{
  ++rock.rec.ret;
  LOG_ERR("Deployment: bad inference #%u (code %d)",
          rock.rec.ret, rock.rec.inf);

  if (rock.rec.ret >= PRE_RECOV_RETRIES + PRE_ABORT_RETRIES)
  {
    rock.state = ABORTED;
    LOG_ERR("FATAL: aborting deployment (%u retries)",
            rock.rec.ret);
  }
  else if (rock.state != RECOVERY &&
           rock.rec.ret >= PRE_RECOV_RETRIES &&
           rock.rec.ret % RECOVERY_INTERVAL == 0)
  {
    rock.rec.state = rock.state;
    rock.state = RECOVERY;
  }
}

/*
 * Invokes state machine and checks for an error.
 * If one exists, invokes appropriate error handler
 * or logs non-critical message.
 *
 * Idle time is configurable in FC-Threads.h.
 */
void deployment_thread_entry(ULONG input)
{
  (void)input;

  LOG_MSG_SYNC("Deployment: starting thread", 28);

  CO2_LOW();
  REEF_LOW();
  rock.state = IDLE;

  while (rock.state != ABORTED) {
    if (rock.state == RECOVERY) {
      try_recover_sensors();
      continue;
    }
    
    rock.rec.inf = infer_rocket_state();

    if (rock.rec.inf < DEPL_OK) {
      bad_inference_handler();
    } else if (rock.rec.inf > DEPL_OK) {
      LOG_ERR("Deployment: warn code %d", rock.rec.inf);
    } else if (rock.rec.ret > 0) {
      rock.rec.ret = 0;
      LOG_MSG("Deployment: recovered from error", 33);
    }

    tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP);
  }
}

/*
 * Creates a non-preemptive deployment thread with
 * defined parameters. Called manually. Provides
 * its entry point with a throwaway input.
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
                             /* No preemption */
                             DEPLOYMENT_THREAD_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    die("Failed to create deployment thread: %u", (unsigned)st);
  }
}