/*
 * Logic related to parachute deployment.
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "FC-Threads.h"
#include "tx_api.h"
#include "tx_port.h"

#include <sedsprintf.h>
#include "telemetry.h"
#include "deployment.h"

/* Thread instance and stack */
TX_THREAD deployment_thread;
ULONG deployment_thread_stack[DEPLOYMENT_THREAD_STACK_SIZE / sizeof(ULONG)];

static rocket_t rock = {0, {0}, {0}, 0, 0};

static inline bool detect_launch(bool confirm) {
  return false;
}

static inline bool detect_burnout() {
  return false;
}

static inline bool detect_apogee(bool confirm) {
  return false;
}

static inline bool detect_reef() {
  return false;
}

static inline bool detect_landed() {
  return false;
}

/* Performs calculations and necessary checks to infer rocket state.
 * Responsible for invoking parachute deployment and triggering abortion. */
static inline bool infer_rocket_state() {

  // TODO gather filter data
  // Verify gathered data: return false on errors
  // For every case, record time (needs clock)

  switch (rock.state) {
    case IDLE:
    {
      if (detect_launch(false)) {
        rock.state = LAUNCH;
        LOG_MSG("Launch detected", 16);
        tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP * 2);
      }
      break;
    }
    case LAUNCH:
    {
      if (detect_launch(true)) {
        rock.state = ASCENT;
        LOG_MSG("Launch confirmed", 17);
      }
    }
    case ASCENT:
    {
      if (detect_burnout()) {
        ++rock.sampl_of.burnout;
        if (rock.sampl_of.burnout >= MIN_BURNOUT) {
          rock.state = BURNOUT;
          LOG_MSG("Watching for apogee", 20);
        }
      }
      break;
    }
    case BURNOUT:
    {
      if (detect_apogee(false)) {
        rock.state = APOGEE;
        // rock.apogee_height = ...
        LOG_MSG("Apogee reached", 15);
        tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP * 2);
      }
      break;
    }
    case APOGEE:
    {
      if (detect_apogee(true)) {
        ++rock.sampl_of.descent;
        if (rock.sampl_of.descent >= MIN_DESCENT) {
          rock.state = DESCENT;
          // fire pyro
          LOG_MSG("Fired pyro, descending", 23);
        }
      }
      break;
    }
    case DESCENT:
    {
      if (detect_reef()) {
        ++rock.sampl_of.landing;
        if (rock.sampl_of.landing >= MIN_REEF) {
          rock.state = REEF;
          // fire reef
          LOG_MSG("Expanded parachute", 19);
        }
      }
      break;
    }
    case REEF:
    {
      if (detect_landed()) {
        ++rock.sampl_of.idle;
        if (rock.sampl_of.idle >= MIN_LANDED) {
          rock.state = LANDED;
          // fire reef
          LOG_MSG("Rocket landed", 14);
        }
      }
      break;
    }
    case LANDED: break;
  }

  return true;
}

/* Entry point of the deployment thread; does not return under normal conditions */
void deployment_thread_entry(ULONG input) {
  (void)input;

  LOG_SYNC("Deployment: starting thread", 28);

  uint16_t retries = 0;
  for (;;) {
    if (!infer_rocket_state()) {
      LOG_ERR("Houston, we have invalid data", 30);
      ++retries;
      
      if (retries >= DEPLOYMENT_THREAD_MAX_RETRIES) {
          LOG_SYNC("FATAL: aborting deployment", 27);
          return;
      }
    } else {  
      retries = 0; // Reset counter (error mitigated)
    }
    tx_thread_sleep(DEPLOYMENT_THREAD_SLEEP);
  }
}

/* Creates a parachute deployment thread with defined parameters */
void create_deployment_thread(void) {
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

  if (st != TX_SUCCESS) {
    die("Failed to create deployment thread: %u", (unsigned)st);
  }
}