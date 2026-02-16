/*
 * Evaluation Task
 * 
 * This task consists of two major parts:
 * KF (Kalman Filters, in kalman.c) and Prediction.
 *
 * The shared ring buffer is actively filled by the
 * Distribution task with compensated raw data taken
 * from DMA buffers (and externally received GPS data
 * if we are running Descent filter). Upon entering
 * its main loop, this task copies the most recent 
 * measurements and optionally validates them.
 *
 * A good, bad, or empty (if validation is disabled)
 * report is then sent to the Recovery task, to maintain
 * heartbeat and change malformed data counter (if any).
 *
 * Then this task updates its local configuration from
 * the global one set by the Recovery task. The config
 * affects speed, operation capacity, verbosity, and
 * precision of both the KF and the Prediction parts.
 * The full list of available Evaluation options can
 * be found in recovery.h. 
 *
 * If valid, data vector is then passed to the either
 * of Ascent or Descent variants of the Kalman Filter.
 *
 * Then, depending on current flight state and run time
 * parameters, the resulting state vector is evaluated by
 * at least one checking function. Some of them require
 * several confirmations of the observation and thus
 * will need several iterations of the Kalman Filter.
 */

#include "evaluation.h"
#include "platform.h"
#include "recovery.h"
#include "kalman.h"
#include "dma.h"

TX_THREAD evaluation_task;
ULONG evaluation_stack[EVAL_STACK_ULONG];


/* ------ Static ------ */

/// Measurement struct but as 1x7 column vector.
static struct measm_z raw = {0};

/// 0-7:  amount of new entries,
/// 8-15: 'consumer lock' index. 0xFF when unlocked.
static atomic_uint_fast16_t mask = 0xFF00u;
static struct measm_z payload[RING_SIZE] = {0};

/// Current and previous state vectors.
static fu8 last = 0;
static struct state_vec vec[2] = {0};

/// Flight stage and transition sample counter.
static enum state flight = Suspended;
static fu8 sampl = 0;


/* ------ Public API ------ */

/// Writes one raw sample to the evaluation ring buffer.
void evaluation_put(const struct measurement *buf)
{
  static fu8 idx = 0;

  fu8 i = idx;
  
  /* Trim temperature and pressure */
  memcpy(&payload[i], buf, sizeof(struct measm_z));

  i = (i + 1) & RING_MASK;
  fu8 cons = load(&mask, Acq) >> 8;

  if (i != cons) {
    fetch_add(&mask, 1, Rel);
    idx = i;
  }
}


/* ------ Local helpers ------ */

static fu8 idx = UINT_FAST8_MAX;

/// Fetches one value from the ring buffer.
/// Returns the amount of new entries added since last call.
static inline fu8 fetch_async(void)
{
  fu16 i = idx;
  fu8 n = (fu8) swap(&mask, i << 8, AcqRel);
  
  if (!n) {
    fetch_or(&mask, CLEAR_IDX, Rlx);
    return 0;
  }

  i = (i + n) & RING_MASK;
  idx = i;

  raw = payload[i];

  fetch_or(&mask, CLEAR_IDX, Rlx);
  return n;
}


/// Fetches one value from the ring buffer.
/// Returns the amount of new entries added since last call.
/// Used when Evaluation cannot be preempted by Distribution.
static inline fu8 fetch_unsafe(void)
{
  fu8 n = (fu8) mask;

  if (!n)
    return 0;

  idx = (idx + n) & RING_MASK;
  raw = payload[idx];

  return n;
}


/// Validates one measm_z against sanity bounds.
static inline fu32 validate(fu32 mode)
{
  enum message st = fc_mask(Sensor_Measm_Code);

  if (!(mode & Validate_Measms)) {
    return st;
  }

  if (raw.d.alt > MAX_ALT || raw.d.alt < MIN_ALT) {
    st += Bad_Altitude;
  }

#ifdef GPS_AVAILABLE
  if (mode & Using_Ascent_KF)
  {
#endif

    if (raw.d.axis.accl.x > MAX_ACC || raw.d.axis.accl.x < MIN_ACC) {
      st += Bad_Accel_X;
    }

    if (raw.d.axis.accl.y > MAX_ACC || raw.d.axis.accl.y < MIN_ACC) {
      st += Bad_Accel_Y;
    }
  
    if (raw.d.axis.accl.z > MAX_ACC || raw.d.axis.accl.z < MIN_ACC) {
      st += Bad_Accel_Z;
    }

    if (raw.gyro.x > MAX_DPS || raw.gyro.x < MIN_DPS) {
      st += Bad_Attitude_X;
    }

    if (raw.gyro.y > MAX_DPS || raw.gyro.y < MIN_DPS) {
      st += Bad_Attitude_Y;
    }

    if (raw.gyro.z > MAX_DPS || raw.gyro.z < MIN_DPS) {
      st += Bad_Attitude_Z;
    }

#if GPS_AVAILABLE
  }
  else if (mode & GPS_Available)
  {
    if (raw.d.axis.gps.x > MAX_GPS_X || raw.d.axis.gps.x < MIN_GPS_X) {
      st += Bad_Lattitude;
    }

    if (raw.d.axis.gps.y > MAX_GPS_Y || raw.d.axis.gps.y < MIN_GPS_Y) {
      st += Bad_Longtitude;
    }
  
    if (raw.d.axis.gps.z > MAX_GPS_Z || raw.d.axis.gps.z < MIN_GPS_Z) {
      st += Bad_Sea_Level;
    }
  }

#endif // GPS_AVAILABLE

  return st;
}


/* ------ Data evaluation ------ */

/// Cautiously examine altitude changes. Act in place.
static inline void evaluate_altitude(fu32 mode)
{
  if (flight < Ascent || vec[last].p.z > vec[!last].p.z) {
    /* Confirms must be consecutive */

    if (mode & static_option(Consecutive_Samples) &&
        mode & static_option(Confirm_Altitude))
    {
      fetch_and(&config, ~static_option(Confirm_Altitude), Rlx);
    }

    return;
  }

  if (vec[last].p.z <= REEF_TARGET_ALT)
  {
    /* We fell below reefing altitude. Depeding on
     * how much we missed, peform the deployments. */

    if (mode & static_option(Parachute_Deployed))
    {
      reef_high();

      if (flight < Reefing) {
        flight = Reefing;
      }
      log_msg("FC:EVAL: urgently expanded reef", 32);
    }
    else
    { 
      co2_high();
      tx_thread_sleep(100);
      reef_high();
      
      if (flight < Reefing) {
        flight = Reefing;
      }

      log_msg("FC:EVAL: urgently fired PYRO->REEF", 35);
    }
  }
  else if (!(mode & static_option(Confirm_Altitude)))
  {
    /* Confirm we are falling before firing CO2. */
    fetch_or(&config, static_option(Confirm_Altitude), Rlx);
  }
  else
  {
    co2_high();

    if (flight < Descent) {
      flight = Descent;
    }

    log_msg("FC:EVAL: urgently fired pyro", 29);
  }
}


/// Monitors if minimum thresholds for velocity and
/// acceleration were exceded.
static inline void detect_launch(void)
{
  if (vec[last].v.z >= LAUNCH_MIN_VEL &&
      vec[last].a.z >= LAUNCH_MIN_VAX)
  {
    flight = Launch;
    log_msg("FC:EVAL: launch detected", 25);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}


/// Monitors height and velocity increase consistency.
static inline void detect_ascent(fu32 mode)
{
  if (vec[last].v.z > vec[!last].v.z &&
      vec[last].p.z > vec[!last].p.z)
  {
    if (++sampl >= MIN_SAMP_ASCENT)
    {
      flight = Ascent;
      sampl = 0;
      log_msg("FC:EVAL: launch confirmed", 26);
    }
  }
  else if (mode & static_option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Launch;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


/// Monitors if minimum threshold for velocity and
/// maximum threshold for acceleration were passed.
/// Checks for height increase and velocity decrease
/// consistency.
static inline void detect_burnout(fu32 mode)
{
  if (vec[last].v.z >= BURNOUT_MIN_VEL &&
      vec[last].a.z <= BURNOUT_MAX_VAX &&
      vec[last].p.z > vec[last].p.z &&
      vec[last].v.z < vec[!last].v.z)
  {
    if (++sampl >= MIN_SAMP_BURNOUT)
    {
      flight = Burnout;
      sampl = 0;
      log_msg("FC:EVAL: watching for apogee", 29);
    }
  }
  else if (mode & static_option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Burnout;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


/// Initially monitors for continuing burnout and
/// for velocity to pass the minimum threshold.
static inline void detect_apogee(void)
{
  if (vec[last].v.z <= APOGEE_MAX_VEL &&
      vec[last].v.z < vec[!last].v.z)
  {
    flight = Apogee;
    log_msg("FC:EVAL: approaching apogee", 28);
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}


/// Monitors for decreasing altitude and increasing velocity.
static inline void detect_descent(fu32 mode)
{
  if (vec[last].p.z < vec[!last].p.z &&
      vec[last].v.z > vec[!last].v.z)
  {
    if (++sampl >= MIN_SAMP_DESCENT)
    {
      flight = Descent;
      sampl = 0;
      co2_high();
      log_msg("FC:EVAL: fired pyro, descending", 32);

      initialize_descent();
      timer_update(DescentKF);
      fetch_and(&config, ~static_option(Using_Ascent_KF), Rel);
    }
  }
  else if (mode & static_option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Descent;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


/// Monitors for falling below a specific altitude,
/// and checks for altitude consistency.
static inline void detect_reef(fu32 mode)
{
  if (vec[last].p.z <= REEF_TARGET_ALT && 
      vec[last].p.z < vec[!last].p.z)
  {
    if (++sampl >= MIN_SAMP_REEF)
    {
      flight = Reefing;
      sampl = 0;
      reef_low();
      log_msg("FC:EVAL: expanded parachute", 28);
    }
  }
  else if (mode & static_option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Reefing;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


/// Monitors that all statistical metrics do not
/// deviate beyond allowed tolerance thresholds.
static inline void detect_landed(fu32 mode)
{
  float dh = vec[last].p.z - vec[!last].p.z;
  float dv = vec[last].v.z - vec[!last].v.z;
  float da = vec[last].a.z - vec[!last].a.z;

  if ((dh <= ALT_TOLER && dh >= -ALT_TOLER) &&
      (dv <= VEL_TOLER && dv >= -VEL_TOLER) &&
      (da <= VAX_TOLER && da >= -VAX_TOLER))
  {
    if (++sampl >= MIN_SAMP_LANDED)
    {
      flight = Landed;
      log_msg("FC:EVAL: rocket landed", 23);
    }
  }
  else if (mode & static_option(Consecutive_Samples)
           && sampl > 0)
  {
    sampl = 0;
    enum message cmd = Not_Landed;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


/* ------ Evaluation Task ------ */

/// High-level overview of data evaluation cycle.
/// This function is idempotent and can be entered twice
/// if critical conditions arise as deemed by Recovery task.
void evaluation_entry(ULONG input)
{
  (void) input;

  fu32 mode = load(&config, Acq);

  if (mode & Launch_Triggered)
  {
     /* Discard unfinished (interrupted) state vector. */
     last = !last;
  }
  else
  {
    log_msg("FC:EVAL: received launch signal", 32);
    flight = Idle;

    /* Signal distribution task to enter main cycle. */
    fetch_or(&config, static_option(Launch_Triggered), Rel);
  }

  if (mode & Using_Ascent_KF) {
    timer_update(AscentKF);
    initialize_ascent();
  } else {
    timer_update(DescentKF);  
    initialize_descent();
  }
  
  task_loop (mode & Eval_Abort_Flag)
  {
    fu32 st = mode & Eval_Focus_Flag ? fetch_unsafe()
                                     : fetch_async();

    if (st == 0) {
      tx_thread_sleep(EVAL_SLEEP_NO_DATA);
      continue;
    }

    st = validate(mode);
    tx_queue_send(&shared, &st, TX_NO_WAIT);

    if (st > 0) {
      tx_thread_relinquish();
      continue;
    }

    fu8 loggable;

#ifdef GPS_AVAILABLE
    if (mode & Using_Ascent_KF) {
      loggable = ASC_STAT * sizeof(float);
      ascentKF (&vec[last], &vec[!last], &raw);
    } else {
      loggable = DESC_STAT * sizeof(float);
      descentKF(&vec[last], &vec[!last], &raw.d);
    }

#else
    loggable = ASC_STAT * sizeof(float);
    ascentKF(&vec[last], &vec[!last], &raw);

#endif

    last = !last;

    log_filter_data(&vec[last], loggable);

    tx_thread_sleep(EVAL_SLEEP_RT_CONF);
    mode = load(&config, Acq);

    if (mode & Monitor_Altitude) {
      evaluate_altitude(mode);
    }

    switch (flight) {
      case Idle:    detect_launch();      break;
      case Launch:  detect_ascent(mode);  break;
      case Ascent:  detect_burnout(mode); break;
      case Burnout: detect_apogee();      break;
      case Apogee:  detect_descent(mode); break;
      case Descent: detect_reef(mode);    break;
      case Reefing: detect_landed(mode);  break;
      default: break;
    }

    tx_thread_relinquish();
    mode = load(&config, Acq);
  }
}


/// Creates a configurably-preemptive, cooperative Evaluation task
/// with defined parameters. This task started by the Recovery task.
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

  if (st != TX_SUCCESS) {
    log_die("FC:EVAL: failed to create evaluation task %u", st);
  }
}