/*
 * Evaluation Task
 * 
 * This task is responsible for the finite state machine
 * and data evaluation. This task suspends on a queue and
 * when woken, evaluates data according to global run time
 * configuration. Depending on the config, this task may or
 * may not be interrupted while doing so. In the case of
 * Ascent filter, this task also runs an update stage and
 * then proceeds to data evaluation.
 * 
 * This task also handles the side effects of state
 * transitions, such as deploying parachute and, if GPS
 * module is enabled and available (those are different),
 * reporting coordinates the rocket landed at.
 *
 * States advance forward from 'Suspended' to 'Landed',
 * with both mentioned being passive states that require
 * interaction, and everything in between being active
 * conditions during which the FC board is autonomous.
 *
 * This task's header file, evalution.h, together with
 * the global configuration found in recovery.h, list a
 * number of options to tune data evaluation. Defined
 * options configure scheduling, confirmation count,
 * consecutiveness, and vigilance of state examination.
 * 
 * This task operates in always-suspend mode, and is
 * resumed by Distribution task when it deems that
 * there is enough raw data to pass.
 */

#include "evaluation.h"
#include "platform.h"
#include "recovery.h"
#include "kalman.h"
#include "dma.h"

TX_SEMAPHORE eval_focus_mode;
TX_THREAD evaluation_task;
ULONG evaluation_stack[EVAL_STACK_ULONG];


/* ------ Global storage ------ */

#define id "EV "

/* History of 4 state vectors and ring index used
 * by Evaluation task and Kalman filter. */
struct state_vec sv[SV_HIST_SIZE] = {0};

struct sv_helper sh = {0};

/* Length of state vector to log, in bytes. Varies per KF. */
fu32 sv_size_bytes = ASC_STAT * sizeof(float);

volatile enum state flight = Suspended;

/* ------ Global storage ------ */


/* ------ Data evaluation ------ */

#define caution " (vigilant)"
#define mlen_caut(len) (mlen(len) + sizeof(caution))

static fu8 sampl = 0;

/*
 * Cautiously examine altitude changes. Act in place.
 * Must be used if 'barometer fallback' mode is enabled.
 */
static inline void evaluate_altitude(fu32 mode)
{
  if (flight < Ascent || sv[sh.idx].p.z > sv[prev(1)].p.z) {
    /* Confirms must be consecutive */

    if (mode & option(Consecutive_Samples) &&
        mode & option(Confirm_Altitude))
    {
      fetch_and(&config, ~option(Confirm_Altitude), Rlx);
    }

    return;
  }

  if (sv[sh.idx].p.z <= REEF_TARGET_ALT)
  {
    /* We fell below reefing altitude. Depeding on
     * how much we missed, peform the deployments. */

    if (mode & option(Parachute_Deployed))
    {
      reef_high(&config);

      if (flight < Reefing) {
        flight = Reefing;
      }
      log_msg(id "fired REEF" caution, mlen_caut(10));
    }
    else
    { 
      co2_high(&config);
      if (load(&config, Acq) & option(Using_Ascent_KF)) {
        initialize_descent();
      }

      tx_thread_sleep(100);
      reef_high(&config);
      
      if (flight < Reefing) {
        flight = Reefing;
      }

      log_msg(id "fired PYRO->REEF" caution, mlen_caut(16));
    }
  }
  else if (!(mode & option(Confirm_Altitude)))
  {
    /* Confirm we are falling before firing CO2. */
    fetch_or(&config, option(Confirm_Altitude), Rlx);
  }
  else
  {
    co2_high(&config);
    if (load(&config, Acq) & option(Using_Ascent_KF)) {
      initialize_descent();
    }

    if (flight < Descent) {
      flight = Descent;
    }

    log_msg(id "fired PYRO" caution, mlen_caut(10));
  }
}

/*
 * Monitors if minimum thresholds for velocity and
 * acceleration were exceded.
 */
static inline void detect_launch(void)
{
  if (sv[sh.idx].v.z >= LAUNCH_MIN_VEL &&
      sv[sh.idx].a.z >= LAUNCH_MIN_VAX)
  {
    flight = Launch;
    log_msg(id "launch detected", mlen(15));
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}

/*
 * Monitors height and velocity increase consistency.
 */
static inline void detect_ascent(fu32 mode)
{
  if (sv[sh.idx].v.z > sv[prev(1)].v.z &&
      sv[sh.idx].p.z > sv[prev(1)].p.z)
  {
    if (++sampl >= MIN_SAMP_ASCENT)
    {
      flight = Ascent;
      sampl = 0;
      log_msg(id "ascending", mlen(9));
    }
  }
  else if (mode & option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Launch;
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
  if (sv[sh.idx].v.z >= BURNOUT_MIN_VEL &&
      sv[sh.idx].a.z <= BURNOUT_MAX_VAX &&
      sv[sh.idx].p.z > sv[sh.idx].p.z &&
      sv[sh.idx].v.z < sv[prev(1)].v.z)
  {
    if (++sampl >= MIN_SAMP_BURNOUT)
    {
      flight = Burnout;
      sampl = 0;
      log_msg(id "decelerating", mlen(12));
    }
  }
  else if (mode & option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Burnout;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Initially monitors for continuing burnout and
 * for velocity to pass the minimum threshold.
 */
static inline void detect_apogee(void)
{
  if (sv[sh.idx].v.z <= APOGEE_MAX_VEL &&
      sv[sh.idx].v.z < sv[prev(1)].v.z)
  {
    flight = Apogee;
    log_msg(id "likely at apogee", mlen(16));
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}

/*
 * Monitors for decreasing altitude and increasing velocity.
 */
static inline void detect_descent(fu32 mode)
{
  if (sv[sh.idx].p.z < sv[prev(1)].p.z &&
      sv[sh.idx].v.z > sv[prev(1)].v.z)
  {
    if (++sampl >= MIN_SAMP_DESCENT)
    {
      flight = Descent;
      sampl = 0;
      co2_high(&config);
      log_msg(id "entered drogue", mlen(14));

      initialize_descent();
    }
  }
  else if (mode & option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Descent;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Monitors for falling below a specific altitude,
 * and checks for altitude consistency.
 */
static inline void detect_reef(fu32 mode)
{
  if (sv[sh.idx].p.z <= REEF_TARGET_ALT && 
      sv[sh.idx].p.z < sv[prev(1)].p.z)
  {
    if (++sampl >= MIN_SAMP_REEF)
    {
      flight = Reefing;
      sampl = 0;
      reef_high(&config);
      log_msg(id "expanded parachute", mlen(18));
    }
  }
  else if (mode & option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Reefing;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/*
 * Monitors that all statistical metrics do not
 * deviate beyond allowed tolerance thresholds.
 */
static inline void detect_landed(fu32 mode)
{
  float dh = sv[sh.idx].p.z - sv[prev(1)].p.z;
  float dv = sv[sh.idx].v.z - sv[prev(1)].v.z;
  float da = sv[sh.idx].a.z - sv[prev(1)].a.z;

  if (fabsf(dh) <= ALT_TOLER && fabsf(dv) <= VEL_TOLER && 
      fabsf(da) <= VAX_TOLER)
  {
    if (++sampl >= MIN_SAMP_LANDED)
    {
      flight = Landed;
      log_msg(id "S.W.E.E.T. landed", mlen(17));

      /* Needed to avoid locks on landing coordinates reporting. */
      enum message cmd = fc_mask(Evaluation_Focus);
      tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
    }
  }
  else if (mode & option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Landed;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/* ------ Data evaluation ------ */


/* ------ Logging ------ */

static inline void
crew_send_coords(fu32 mode)
{
#ifdef GPS_AVAILABLE
  if (!(mode & option(GPS_Available))) {
    return;
  }

  log_msg(id "Landed at coordinates:", mlen(22));
  log_measurement(SEDS_DT_GPS_DATA, &payload.d.gps);

  tx_thread_sleep(LANDED_GPS_INTERVAL);

#else
  return;

#endif // GPS_AVAILABLE
}

/* ------ Logging ------ */


/* ------ Evaluation Task ------ */

/*
 * Simple finite-state machine for state transition.
 * Before leaving, logs state vector just used.
 */
blind_inline void evaluate_rocket_state(fu32 conf)
{
  if (conf & Monitor_Altitude) {
    evaluate_altitude(conf);
  }

  switch (flight) {
    case Idle:    detect_launch();        break;
    case Launch:  detect_ascent(conf);    break;
    case Ascent:  detect_burnout(conf);   break;
    case Burnout: detect_apogee();        break;
    case Apogee:  detect_descent(conf);   break;
    case Descent: detect_reef(conf);      break;
    case Reefing: detect_landed(conf);    break;
    case Landed:  crew_send_coords(conf); break;
    default: break;
  }

  log_filter_data(&sv[sh.idx], sv_size_bytes);

  /* Increment index for the next state vector */
  sh.idx = (sh.idx + 1) & SV_HIST_MASK;
}

/*
 * Initializes Ascent filter and signal Distribution
 * task to start performing data logistics. Idempotent.
 * If there is a re-entrancy, discards dirty KF buffers.
 */
static inline void
enter_flight_state(fu32 conf)
{
  if (conf & Launch_Triggered)
  {
     /* Discard unfinished (interrupted) state vector. */
     sh.idx = prev(1);
  }
  else
  {
    log_msg(id "received launch signal", mlen(22));
    flight = Idle;

    initialize_ascent();

    /* Signal distribution task to enter main cycle. */
    fetch_or(&config, option(Launch_Triggered), Rel);
  }
}

/*
 * Suspends on a message queue and performs
 * data evaluation in accordance with global config.
 */
void evaluation_entry(ULONG input)
{
  (void) input;

  UINT st;

  fu32 conf = load(&config, Acq);

  enter_flight_state(conf);
  
  task_loop (conf & Eval_Abort_Flag)
  {
    /* Task suspension */
    st = tx_semaphore_get(&eval_focus_mode, TX_WAIT_FOREVER);

    if (st != TX_SUCCESS) {
      continue;
    }

    conf = load(&config, Acq);

    if (conf & Using_Ascent_KF)
    {
      update(sh.dt);
    }

    evaluate_rocket_state(conf);
  }
}


/*
 * Creates a configurably-preemptive, cooperative Evaluation task
 * with defined parameters. This task started by the Recovery task.
 */
void create_evaluation_task(void)
{
  UINT st = tx_thread_create(&evaluation_task,
                             "Evaluation Task",
                             evaluation_entry,
                             EVAL_INPUT,
                             evaluation_stack,
                             EVAL_STACK_BYTES,
                             EVAL_PRIORITY,
                             EVAL_PREEMPT_THRESHOLD,
                             EVAL_TIME_SLICE,
                             TX_DONT_START);

  const char *critical = "creation failure:";

  if (st != TX_SUCCESS) {
    log_die(id "task %s %u", critical, st);
  }

  st = tx_semaphore_create(&eval_focus_mode, "EVALS", 0);

  if (st != TX_SUCCESS) {
    log_die(id "sema %s %u", critical, st);
  }
}

/* ------ Evaluation Task ------ */