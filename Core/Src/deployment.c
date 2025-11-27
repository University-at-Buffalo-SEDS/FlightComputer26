/*
 * Logic related to state monitoring and parachute deployment.
 */

#include "deployment.h"
#include "tx_api.h"
#include <stdatomic.h>
#include <stdint.h>

TX_THREAD deployment_thread;
ULONG deployment_thread_stack[DEPLOYMENT_THREAD_STACK_SIZE / sizeof(ULONG)];

static rocket_t rock = {0, {0}, {0}, {0}};
static filter_t curr[DEPL_BUF_SIZE] = {0};
static filter_t prev[DEPL_BUF_SIZE] = {0};
extern atomic_uint_fast16_t newdata;

// TODO add ring and validate logic once kalman api exposed

/*
 * Copies "current" data into "previous" array and
 * populates "current" with new data from the filter ring.
 *
 * rock.i.klm (k) - index of last copied element from the ring,
 * rock.i.cur (d) - the current "size" of "current" data,
 * rock.i.old     - current "size" of "previous" data.
 *
 * "size" is the amount of active elements in a local buffer.
 * newdata (n) is incremented by the filter each time it writes
 * a new element to the ring. It is atomic to avoid conflicts.
 *
 * When "current" is copied into "previous," the "size"
 * of "current" is inherited by "previous". It helps to
 * avoid using "previous-previous" garbage data in calculations.
 *
 * Then we compute the starting point in the ring. If newdata is
 * smaller than or the same as the size of our buffer, we leave
 * it as it is. If it is bigger, we advance our starting position
 * in order to copy the latest of the available new data.
 *
 * Every iteration we lock a specific element of the filter ring
 * to avoid partial reads, and to let filter write other elements.
 * 
 * If the deployment task is switched to filter task while
 * the second loop was running, newdata will grow. If the filter
 * encounters our lock, it should starve or sched_yield.
 * 
 * When the deployment task is back, it will continue to use its
 * old copy of newdata, and by doing so it will stop exactly
 * before the first element that was added during the interruption.
 * When this function is called the next time, it will advance its
 * starting position again to catch up with faster producer.
 */
static inline inference_e refresh_data()
{
  uint_fast16_t n;

  if (!(n = atomic_exchange_explicit(&newdata, 0, memory_order_acq_rel))) {
    // tx_thread_resume(&kalman_thread);
    return DEPL_NO_INPUT;
  }

  uint_fast16_t d = 0;
  uint_fast16_t k = rock.i.klm;

  // if (n > DEPL_BUF_SIZE)
  //   k = (k + n - DEPL_BUF_SIZE) & (KALMAN_RING_SIZE - 1);

  for (; d < rock.i.cur; ++d) {
    prev[d] = curr[d];
  }

  for (d = 0; d < MIN(n, DEPL_BUF_SIZE); ++d) {
    // k = (k + 1) & (KALMAN_RING_SIZE - 1);
    // lock kalman_ring[k]
    // curr[d] = kalman_ring[rock.i.klm];
    // unlock kalman_ring[k]
  }

  rock.i.klm = k;
  rock.i.prv = rock.i.cur;
  rock.i.cur = d;

  return DEPL_OK;
}

static inline inference_e detect_launch(inference_e confirm)
{
  return DEPL_OK;
}

static inline inference_e detect_burnout()
{
  return DEPL_OK;
}

static inline inference_e detect_apogee(inference_e confirm)
{
  return DEPL_OK;
}

static inline inference_e detect_reef()
{
  return DEPL_OK;
}

static inline inference_e detect_landed()
{
  return DEPL_OK;
}

/*
 * This is a state machine that calls functions necessary
 * to gather, validate, process, and draw inference from data.
 *
 * Current implementation does not allow state regression
 * and triggers checks to prevent premature or wrong triggers.
 *
 * Timings and amount of checks are configurable in deployment.h.
 */
static inline inference_e infer_rocket_state()
{
  if (refresh_data() == DEPL_NO_INPUT)
    return DEPL_OK;  

  // TODO
  // Verify gathered data: return false on errors
  // For every case, record time (needs clock)

  switch (rock.state)
  {
    case IDLE:
    {
      if (detect_launch(INFER_INITIAL))
      {
        rock.state = LAUNCH;
        LOG_MSG("Launch detected", 16);
        WAIT_BEFORE_CONFIRM();
      }
      break;
    }
    case LAUNCH:
    {
      if (detect_launch(INFER_CONFIRM))
      {
        rock.state = ASCENT;
        rock.samp_of.burnout = 0;
        LOG_MSG("Launch confirmed", 17);
      }
      break;
    }
    case ASCENT:
    {
      if (detect_burnout())
      {
        ++rock.samp_of.burnout;
        if (rock.samp_of.burnout >= MIN_SAMP_BURNOUT)
        {
          rock.state = BURNOUT;
          rock.samp_of.descent = 0;
          LOG_MSG("Watching for apogee", 20);
        }
      }
      break;
    }
    case BURNOUT:
    {
      if (detect_apogee(INFER_INITIAL))
      {
        rock.state = APOGEE;
        LOG_MSG("Apogee reached", 15);
        WAIT_BEFORE_CONFIRM();
      }
      break;
    }
    case APOGEE:
    {
      if (detect_apogee(INFER_CONFIRM))
      {
        ++rock.samp_of.descent;
        if (rock.samp_of.descent >= MIN_SAMP_DESCENT)
        {
          rock.state = DESCENT;
          rock.samp_of.landing = 0;
          CO2_HIGH();
          LOG_MSG("Fired pyro, descending", 23);
        }
      }
      break;
    }
    case DESCENT:
    {
      if (detect_reef())
      {
        ++rock.samp_of.landing;
        if (rock.samp_of.landing >= MIN_SAMP_REEF)
        {
          rock.state = REEF;
          rock.samp_of.idle = 0;
          REEF_HIGH();
          LOG_MSG("Expanded parachute", 19);
        }
      }
      break;
    }
    case REEF:
    {
      if (detect_landed())
      {
        ++rock.samp_of.idle;
        if (rock.samp_of.idle >= MIN_SAMP_LANDED)
        {
          rock.state = LANDED;
          LOG_MSG("Rocket landed", 14);
        }
      }
      break;
    }
    case LANDED: break;
  }

  return DEPL_OK;
}

/*
 * Entry point of the deployment thread. Usually called by RTOS.
 *
 * Returns (aborts thread) if the state machine returned an error
 * certain amount of times in a row (that is, if the error)
 * could not be mitigated within a given time.
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
    if ((last = infer_rocket_state()) != DEPL_OK)
    {
      LOG_ERR("Deployment: bad inference #%u (code %d)", retries, last);
      ++retries;

      if (retries >= DEPLOYMENT_THREAD_MAX_RETRIES)
      {
        LOG_ERR_SYNC("FATAL: aborting deployment (%u retries)", retries);
        return;
      }
    }
    else
    {
      retries = 0; // Reset counter (error mitigated)
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