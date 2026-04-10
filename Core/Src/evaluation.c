/*
 * Evaluation Task
 */

#include "platform.h"
#include "fctypes.h"
#include "fcstructs.h"
#include "fctasks.h"
#include "fccommon.h"
#include "fcapi.h"
#include "fcconfig.h"
#include "sweetbench.h"

#define id "EV "
#define id_vigilant "VM "


TX_THREAD evaluation_task;
TX_SEMAPHORE eval_focus_mode;

kf_svec sv[STATE_HISTORY] = {0};
sv_meta sm = {0};

volatile state flight = Suspended;

const char *trans[Flight_States] = {
    [Suspended] = " interval in pilot mode:",
    [Idle]      = "",
    [Launch]    = "Launch detected. Acceleration in Z:",
    [Ascent]    = "Ascending. Velocity in Z:",
    [Burnout]   = "Decelerating. Altitude:",
    [Apogee]    = "Approaching apogee. Altitude:",
    [Descent]   = "Descending in drogue. Altitude:",
    [Reefing]   = "Expanded parachute. Altitude:",
    [Landed]    = "Landed. Coordinates will follow.",
};


/*
 * Vigilant mode routine. Can perform urgent deployments
 * and skip states.
 */
static inline void evaluate_altitude(fu32 mode)
{
  if (flight < Ascent || svec(0).alt > svec(1).alt)
  {
    if (mode & option(Consecutive_Samples) &&
        mode & option(Confirm_Altitude))
    {
      fetch_and(&g_conf, ~option(Confirm_Altitude), Rlx);
    }

    return;
  }

  if (svec(0).alt <= REEF_TARGET_ALT &&
      !(mode & option(Parachute_Expanded)))
  {
    /* We fell below reefing altitude. Depeding on
     * how much we missed, peform the deployments. */

    if (mode & option(Parachute_Deployed))
    {
      expand_parachute();

      flight = Reefing;
      log_transition(id_vigilant, svec(0).alt);
    }
    else
    {
      release_parachute();

      flight = Descent;
      log_transition(id_vigilant, svec(0).alt);

      descent_initialize();

      tx_thread_sleep(URGENT_DEPLOYMENT_DELAY);
      expand_parachute();

      flight = Reefing;
      log_transition(id_vigilant, svec(0).alt);
    }
  }
  else if (!(mode & option(Confirm_Altitude)))
  {
    fetch_or(&g_conf, option(Confirm_Altitude), Rlx);
  }
  else if (!(mode & option(Parachute_Deployed)))
  {
    release_parachute();
    
    flight = Descent;
    log_transition(id_vigilant, svec(0).alt);
    
    descent_initialize();
  }
}

/*
 * Monitors if minimum thresholds for velocity and
 * acceleration were exceded.
 */
static inline void detect_launch(void)
{
  if (svec(0).vel >= LAUNCH_MIN_VEL &&
      svec(0).tail.asc.acz >= LAUNCH_MIN_VAX)
  {
    flight = Launch;
    log_transition(id, svec(0).tail.asc.acz);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}

/*
 * Monitors height and velocity increase consistency.
 */
static inline void detect_ascent(fu32 mode)
{
  if (svec(0).vel > svec(1).vel &&
      svec(1).vel > svec(2).vel &&
      svec(0).alt > svec(1).alt &&
      svec(1).alt > svec(2).alt)
  {
    if (++sm.samp >= MIN_SAMP_ASCENT)
    {
      flight = Ascent;
      sm.samp = 0;
      log_transition(id, svec(0).vel);
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Launch;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Monitors if minimum threshold for velocity and
 * maximum threshold for acceleration were passed.
 * Checks for height increase and velocity decrease
 * consistency.
 */
static inline void detect_burnout(fu32 mode)
{
  if (svec(0).vel >= BURNOUT_MIN_VEL &&
      svec(0).tail.asc.acz <= BURNOUT_MAX_VAX &&
      svec(0).alt > svec(1).alt &&
      svec(0).vel < svec(1).vel)
  {
    if (++sm.samp >= MIN_SAMP_BURNOUT)
    {
      flight = Burnout;
      sm.samp = 0;
      log_transition(id, svec(0).alt);
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Burnout;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Initially monitors for continuing burnout and
 * for velocity to pass the minimum threshold.
 */
static inline void detect_apogee(void)
{
  if (svec(0).vel <= APOGEE_MAX_VEL &&
      svec(0).vel < svec(1).vel &&
      svec(1).vel < svec(2).vel &&
      svec(2).vel < svec(3).vel)
  {
    flight = Apogee;
    log_transition(id, svec(0).alt);
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}

/*
 * Monitors for decreasing altitude and increasing velocity.
 */
static inline void detect_descent(fu32 mode)
{
  if (svec(0).alt < svec(1).alt &&
      svec(1).alt < svec(2).alt &&
      svec(0).vel > svec(1).vel &&
      svec(1).vel > svec(2).vel)
  {
    if (++sm.samp >= MIN_SAMP_DESCENT)
    {
      flight = Descent;
      sm.samp = 0;
      release_parachute();
      log_transition(id, svec(0).alt);

      descent_initialize();
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Descent;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Monitors for falling below a specific altitude,
 * and checks for altitude consistency.
 */
static inline void detect_reef(fu32 mode)
{
  if (svec(0).alt <= REEF_TARGET_ALT &&
      svec(0).alt < svec(1).alt)
  {
    if (++sm.samp >= MIN_SAMP_REEF)
    {
      flight = Reefing;
      sm.samp = 0;
      expand_parachute();
      log_transition(id, svec(0).alt);
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Reefing;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Monitors that all statistical metrics do not
 * deviate beyond allowed tolerance thresholds.
 */
static inline void detect_landed(fu32 mode)
{
  float dh = svec(0).alt - svec(1).alt;
  float dv = svec(0).vel - svec(1).vel;

  if (fabsf(dh) <= ALT_TOLER && fabsf(dv) <= VEL_TOLER)
  {
    if (++sm.samp >= MIN_SAMP_LANDED)
    {
      flight = Landed;
      log_msg(trans[Landed]);

      fc_msg cmd = fc_mask(Evaluation_Focus);
      tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
    }
  }
  else if (mode & option(Consecutive_Samples) && sm.samp > 0)
  {
    sm.samp = 0;
    fc_msg cmd = Not_Landed;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


static inline void crew_send_coords(fu32 mode)
{
#ifdef GPS_AVAILABLE
  if (!(mode & option(GPS_Available)))
  {
    return;
  }

  log_measm(SEDS_DT_GPS_DATA, &meas.mode.gps);

  tx_thread_sleep(LANDED_GPS_INTERVAL);

#else
  return;

#endif /* GPS_AVAILABLE */
}

/*
 * Finite-state machine for state transition.
 * Before leaving, logs state vector just used.
 */
void evaluate_rocket_state(fu32 conf)
{
  if (conf & Monitor_Altitude)
  {
    evaluate_altitude(conf);
  }

  switch (flight)
  {
  case Idle:
    detect_launch();
    break;
  case Launch:
    detect_ascent(conf);
    break;
  case Ascent:
    detect_burnout(conf);
    break;
  case Burnout:
    detect_apogee();
    break;
  case Apogee:
    detect_descent(conf);
    break;
  case Descent:
    detect_reef(conf);
    break;
  case Reefing:
    detect_landed(conf);
    break;
  case Landed:
    crew_send_coords(conf);
    break;
  default:
    break;
  }

  fu32 size = conf & option(Using_Ascent_KF) ? ASC_STAT
                                             : DESC_STAT;

  log_filter_data(&svec(0), size);

  sm.idx = (sm.idx + 1) & STATE_HISTORY_MASK;
}

/*
 * Initializes Ascent filter and signals Distribution
 * task to start performing data logistics. Idempotent.
 * If there is a re-entrancy, discards dirty KF buffer.
 */
static inline void enter_flight_state(fu32 conf)
{
  if (conf & option(Launch_Triggered))
  {
    sm.idx = (sm.idx - 1) & STATE_HISTORY_MASK;
  }
  else
  {
    ascent_initialize();
    log_msg(id "received launch signal");

    flight = Idle;
    
    fetch_or(&g_conf, option(Launch_Triggered), Rel);
  }
}

/*
 * Suspends on a message queue and performs
 * data evaluation in accordance with global config.
 */
void evaluation_entry(ULONG input)
{
  (void)input;

  log_msg(id "started");

  UINT st;

  fu32 conf = load(&g_conf, Acq);

  enter_flight_state(conf);

  task_loop (conf & option(Eval_Abort_Flag))
  {
    st = tx_semaphore_get(&eval_focus_mode, TX_WAIT_FOREVER);

    if (st != TX_SUCCESS)
    {
      continue;
    }

    ascent_update(sm.dt);

    conf = load(&g_conf, Acq);

    evaluate_rocket_state(conf);
  }
}

/*
 * Creates a configurably-preemptive, cooperative Evaluation task
 * with defined parameters. Started by the Recovery task.
 */
UINT create_evaluation_task(TX_BYTE_POOL *byte_pool)
{
  UINT st;
  CHAR *pointer;

  if (tx_byte_allocate(byte_pool, (VOID **)&pointer,
                       EVAL_STACK_BYTES, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }

  st = tx_thread_create(&evaluation_task,
                        "Evaluation Task",
                        evaluation_entry,
                        EVAL_INPUT,
                        pointer,
                        EVAL_STACK_BYTES,
                        EVAL_PRIORITY,
                        EVAL_PREEMPT_THRESHOLD,
                        EVAL_TIME_SLICE,
                        TX_DONT_START);

  const char *critical = "creation failure:";

  if (st != TX_SUCCESS)
  {
    log_die(id "task %s %u", critical, st);
  }

  st = tx_semaphore_create(&eval_focus_mode, "EVALS", 0);

  if (st != TX_SUCCESS)
  {
    log_die(id "sema %s %u", critical, st);
  }

  return TX_SUCCESS;
}