/*
 * Evaluation Task
 * 
 * This task consists of two major parts:
 * KF (Kalman Filter, in kalman.c) and Prediction.
 *
 * The shared ring buffer is actively filled by the
 * Distribution task with compensated raw data taken
 * from DMA buffers. Upon entering its main loop,
 * this task copies the most recently available pack
 * of measurements (baro, gyro, accel), and checks
 * them against sanity boundaries. Regardless of the
 * outcome of this evaluation, a status code (a success
 * or bad data report) is passed to the Recovery Task
 * queue in order to maintain internal FC heartbeat.
 * If there was bad data, the task will take another
 * (always newer) buffer from the ring or wait if 
 * there is no one available (unlikely).
 *
 * Then this task updates its local configuration
 * from the global one (for later non-atomic access),
 * and evaluates certain options if possible (such as
 * FORCE_ALT_CHECKS, which direct to always monitor
 * altitude changes, regardless of current state).
 *
 * Depending on whether apogee is reached, the raw
 * and presumably valid data is passed to the either
 * of Ascent or Descent variants of the Kalman Filter.
 * This is where this task will be stuck for a while,
 * despite my attempts to use fast math and maintain
 * cache coherency in math operations. This is why
 * UKF and Prediction tasks are combined in sequential
 * logic: losing one processed vector due to async
 * would be timewise expensive.
 *
 * Then, depending on current flight state and run time
 * parameters, the processed vector is evaluated using
 * one of the checking functions. Some of them require
 * several confirmations of the observation and thus
 * will need several iterations of the Kalman Filter.
 *
 * Unless Earth's gravity becomes like that on Jupiter,
 * we don't need to rush here!
 */

#include "evaluation.h"
#include "platform.h"
#include "recovery.h"
#include "kalman.h"
#include "dma.h"

TX_THREAD evaluation_task;
ULONG evaluation_stack[EVAL_STACK_ULONG];

/// Module config bitmask. Global as used by Kalman filter.
fu16 mode = 0;

/// Separate flag in addition to flight state
/// to avoid repeated comparisons in handlers.
atomic_uint_fast8_t ascending = 1;


/* ------ Static ------ */

/// Raw data ring.
static struct measm_z payload[RING_SIZE] = {0};

/// 0-7:  amount of new entries,
/// 8-15: 'consumer locked' index
/// The latter is 255 (FF) when consumer is not reading the buffer.
static atomic_uint_fast16_t mask = 0xFF00u;

/// Last state vector buffer {0, 1}
static fu8 last = 0;

/// Amount of successive samples per flight stage
static fu8 sampl = 0;

/// Current flight state
static enum state flight = SUSPENDED;

/* Addresses of the following structs and their 
 * members are constants. When passed to external
 * functions, address arguments are subjected to
 * compiler and linker optimizations. */

/// Current and previous state vectors
static struct state_vec vec[2] = {0};

/// Latest local measurement (trimmed of temperature and pressure)
static struct measm_z raw = {0};


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
static inline fu8 fetch_async()
{
  fu16 i = idx;

  /* Fetch and clear new entries counter
   * and store current index */
  fu8 n = (fu8) swap(&mask, i << 8, AcqRel);
  
  if (!n)
    goto finish;

  i = (i + n) & RING_MASK;
  idx = i;

  raw = payload[i];

finish:
  /* Always clear busy index (relaxed) */
  fetch_or(&mask, CLEAR_IDX, Rlx);
  return n;
}

/// Fetches one value from the ring buffer.
/// This version should only be used when preemption for
/// the Evaluation task is disabled.
/// Returns the amount of new entries added since last call.
static inline fu8 fetch_unsafe()
{
  fu8 n = (fu8) mask;

  if (!n)
    return 0;

  idx = (idx + n) & RING_MASK;
  raw = payload[idx];

  return n;
}

#ifdef MEASM_VALIDATE

/// Validates one raw measurement against sanity bounds.
static inline fu8
validate()
{
  enum command st = RAW_DATA;

  if (raw.d.alt > MAX_ALT || raw.d.alt < MIN_ALT)
    st += RAW_BAD_ALT;

#if defined (TELEMETRY_ENABLED) && defined (GPS_AVAILABLE)
  if (ascending)
  {
#endif // TELEMETRY_ENABLED * GPS_AVAILABLE

    if (raw.d.axis.accl.x > MAX_VAX || raw.d.axis.accl.x < MIN_VAX)
      st += RAW_BAD_ACC_X;

    if (raw.d.axis.accl.y > MAX_VAX || raw.d.axis.accl.y < MIN_VAX)
      st += RAW_BAD_ACC_Y;
  
    if (raw.d.axis.accl.z > MAX_VAX || raw.d.axis.accl.z < MIN_VAX)
      st += RAW_BAD_ACC_Z;

#if defined (TELEMETRY_ENABLED) && defined (GPS_AVAILABLE)
  }
  else
  {
    if (raw.d.axis.gps.x > MAX_GPS_X || raw.d.axis.gps.x < MIN_GPS_X)
      st += RAW_BAD_GPS_X;

    if (raw.d.axis.gps.y > MAX_GPS_Y || raw.d.axis.gps.y < MIN_GPS_Y)
      st += RAW_BAD_GPS_Y;
  
    if (raw.d.axis.gps.z > MAX_GPS_Z || raw.d.axis.gps.z < MIN_GPS_Z)
      st += RAW_BAD_GPS_Z;
  }

#endif // TELEMETRY_ENABLED * GPS_AVAILABLE
  
  if (raw.gyro.x > MAX_ANG || raw.gyro.x < MIN_ANG)
    st += RAW_BAD_ANG_X;

  if (raw.gyro.y > MAX_ANG || raw.gyro.y < MIN_ANG)
    st += RAW_BAD_ANG_Y;

  if (raw.gyro.z > MAX_ANG || raw.gyro.z < MIN_ANG)
    st += RAW_BAD_ANG_Z;

  st = FC_MSG(st);
  /*
   * Even if data has been successfully validated,
   * send 'OK' signal in order to maintain heartbeat.
   */
  tx_queue_send(&shared, &st, TX_NO_WAIT);

  return st;
}

#endif // MEASM_VALIDATE


/* ------ Data evaluation ------ */

/// Cautiously examine altitude changes
/// in order to make an immediate decision.
static inline void
evaluate_altitude()
{
  /* If we aren't yet ascending or not descending, return.
   * First condition is to guard against noise while idle. */
  if (flight < ASCENT || vec[last].p.z > vec[!last].p.z) {
    /* If we were waiting on confirmation BUT config mandates
     * confirmations to be consecutive, reset the flag */
    if (mode & CONSECUTIVE_SAMP && mode & PYRO_REQ_CONFIRM) {
      mode &= ~PYRO_REQ_CONFIRM;
    }

    return;
  }

  if (vec[last].p.z <= REEF_TARGET_ALT)
  {
    /* We somehow missed the moment to expand parachute.
      * Do NOT release control until reef is fired */
    if (mode & SAFE_EXPAND_REEF)
    {
      reef_high();
      log_msg("FC:EVAL: urgently expanding reef", 33);
    }
    else
    {
      /* We didn't even expand parachute to that moment.
      * What is going on!!? Perform full sequence. */ 
      co2_high();
      log_msg("FC:EVAL: urgently firing PYRO->REEF", 36);
      mode |= SAFE_EXPAND_REEF;
      tx_thread_sleep(100); // sleep for 1 sec
      reef_high();
    }
  }
  else if (!(mode & PYRO_REQ_CONFIRM))
  {
    /* Request one confirmation of decreasing height,
      * then fire PYRO. */
    mode |= PYRO_REQ_CONFIRM;
  }
  else
  {
    co2_high();
    mode |= SAFE_EXPAND_REEF;
  }
}

/// Monitors if minimum thresholds for velocity and
/// acceleration were exceded.
static inline void
detect_launch()
{
  if (vec[last].v.z >= LAUNCH_MIN_VEL &&
      vec[last].a.z >= LAUNCH_MIN_VAX)
  {
    flight = LAUNCH;
    log_msg("FC:EVAL: launch detected", 25);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}

/// Monitors height and velocity increase consistency.
static inline void
detect_ascent()
{
  if (vec[last].v.z > vec[!last].v.z &&
      vec[last].p.z > vec[!last].p.z)
  {
    if (++sampl >= MIN_SAMP_ASCENT)
    {
      flight = ASCENT;
      sampl = 0;
      log_msg("FC:EVAL: launch confirmed", 26);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && sampl > 0)
  {
    sampl = 0;
    enum command cmd = NOT_LAUNCH;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Monitors if minimum threshold for velocity and
/// maximum threshold for acceleration were passed.
/// Checks for height increase and velocity decrease
/// consistency.
static inline void
detect_burnout()
{
  if (vec[last].v.z >= BURNOUT_MIN_VEL &&
      vec[last].a.z <= BURNOUT_MAX_VAX &&
      vec[last].p.z > vec[last].p.z &&
      vec[last].v.z < vec[!last].v.z)
  {
    if (++sampl >= MIN_SAMP_BURNOUT)
    {
      flight = BURNOUT;
      sampl = 0;
      log_msg("FC:EVAL: watching for apogee", 29);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && sampl > 0)
  {
    sampl = 0;
    enum command cmd = NOT_BURNOUT;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Initially monitors for continuing burnout and
/// for velocity to pass the minimum threshold.
static inline void
detect_apogee()
{
  if (vec[last].v.z <= APOGEE_MAX_VEL &&
      vec[last].v.z < vec[!last].v.z)
  {
    flight = APOGEE;
    log_msg("FC:EVAL: approaching apogee", 28);
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}

/// Monitors for decreasing altitude and increasing velocity.
static inline void
detect_descent()
{
  if (vec[last].p.z < vec[!last].p.z &&
      vec[last].v.z > vec[!last].v.z)
  {
    if (++sampl >= MIN_SAMP_DESCENT)
    {
      flight = DESCENT;
      sampl = 0;
      co2_high();
      log_msg("FC:EVAL: fired pyro, descending", 32);

#if defined (TELEMETRY_ENABLED) && defined (GPS_AVAILABLE)
      if (!(load(&config, Acq) & USE_ASCENT)) {
        initialize_descent();
        ascending = 0;
      }

#endif // TELEMETRY_ENABLED * GPS_AVAILABLE
    }
  }
  else if (mode & CONSECUTIVE_SAMP && sampl > 0)
  {
    sampl = 0;
    enum command cmd = NOT_DESCENT;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Monitors for falling below a specific altitude,
/// and checks for altitude consistency.
static inline void
detect_reef()
{
  if (vec[last].p.z <= REEF_TARGET_ALT && 
      vec[last].p.z < vec[!last].p.z)
  {
    if (++sampl >= MIN_SAMP_REEF)
    {
      flight = REEF;
      sampl = 0;
      reef_low();
      log_msg("FC:EVAL: expanded parachute", 28);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && sampl > 0)
  {
    sampl = 0;
    enum command cmd = NOT_REEF;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Monitors all statistical metrics to not deviate
/// beyond allowed tolerance thresholds.
static inline void
detect_landed()
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
      flight = LANDED;
      log_msg("FC:EVAL: rocket landed", 23);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && sampl > 0)
  {
    sampl = 0;
    enum command cmd = NOT_LANDED;
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

  if (load(&config, Acq) & ENTER_DIST_CYCLE)
  {
    /* This is a re-entry. Discard the buffer processing
     * which the system timed out. KF will receive older
     * state vector and recompute new state vec into the
     * buffer that was interrupted. */
     last = !last;
  }
  else /* Initial launch command */
  {
    log_msg("FC:EVAL: received launch signal", 32);
    flight = IDLE;

    /* Signal distribution task to enter main cycle */
    fetch_or(&config, ENTER_DIST_CYCLE, Rel);
  }

  /* Begin counting down elapsed time for 
   * KF kinematic equations (predict function) */
  timer_update(ascending ? AscentKF : DescentKF);  
  
  task_loop (mode & ABORT_EVALUATION)
  {
    /* Fetch & validate operate on static buffers => no args. */

    fu8 st = mode & EVAL_PREEMPT_OFF ? fetch_unsafe()
                                     : fetch_async();

    if (st == 0) {
      tx_thread_sleep(EVAL_SLEEP_NO_DATA);
      continue;
    }

#ifdef MEASM_VALIDATE
    st = validate();

    if (st > 0) {
      /* Depending on comptime config (sleep intervals
       * and thread time slices), relinquishing may
       * return control back to this thread faster than
       * sleeping. Here, we retry not because the producer
       * is slow, but because the data is invalid. Thus
       * we want producer to run for its minimum (1 slice). */
      tx_thread_relinquish();
      continue;
    }

#endif // MEASM_VALIDATE

    /* One state vector is input (x_0), other is for output only (x_f) */
#if defined (TELEMETRY_ENABLED) && defined (GPS_AVAILABLE)
    ascending ? ascentKF (&vec[last], &vec[!last], &raw)  :
                descentKF(&vec[last], &vec[!last], &raw.d);

#else
    ascentKF(&vec[last], &vec[!last], &raw);

#endif // TELEMETRY_ENABLED * GPS_AVAILABLE

    last = !last;

    /* ^^^ Log latest state vec excluding quaternions */
    log_filter_data(&vec[last], STATE_LOGGABLE);

    /* Long cycle of Distribution and Telemetry tasks */
    tx_thread_sleep(EVAL_SLEEP_RT_CONF);

    /* When back, load any changes in config */
    mode = load(&config, Acq);

    if (mode & FORCE_ALT_CHECKS) {
      evaluate_altitude();
    }

    switch (flight) {
      case IDLE:
        detect_launch();
        break;
      case LAUNCH:
        detect_ascent();
        break;
      case ASCENT:
        detect_burnout();
        break;
      case BURNOUT:
        detect_apogee();
        break;
      case APOGEE:
        detect_descent();
        break;
      case DESCENT:
        detect_reef();
        break;
      case REEF:
        detect_landed();
        break;
      default: break;
    }

    /* Short cycle of Distribution and Telemetry tasks */
    tx_thread_relinquish();

    /* When back, load any changes in config */
    mode = load(&config, Acq);
  }
}

/// Creates a configurably-preemptive, cooperative
/// Evaluation task with defined parameters.
/// This task does not start automatically and is
/// instead resumer by the Recovery task.
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