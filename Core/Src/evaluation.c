/*
 * Evaluation Task
 * 
 * This task consists of two major parts:
 * UKF (Unscented Kalman Filter) and Prediction.
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
 * cache friednliness in math operations. This is why
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

#include <stdint.h>
#include <stdatomic.h>

#include "dsp/fast_math_functions.h"

#include "evaluation.h"
#include "platform.h"
#include "recovery.h"
#include "dma.h"

TX_THREAD evaluation_task;
ULONG evaluation_stack[EVAL_STACK_ULONG];

/// Current flight state
enum state flight = SUSPENDED;


/* ------ Static ------ */

/// Raw data ring.
static struct measurement payload[RING_SIZE] = {0};

/// 0-7:  amount of new entries,
/// 8-15: 'consumer locked' index
/// The latter is 255 (FF) when consumer doesn't
/// is not reading the buffer.
static atomic_uint_fast16_t mask = 0xFF00u;

/// Local module config bitmask
static fu16 mode = 0;


/* ------ Public API ------ */

/// Aggregates sensor compensation functions.
/// Run before reporting and publishing data.
void compensate(struct measurement *buf)
{
  buf->baro.t   = compensate_temperature((uint32_t)buf->baro.t);
  buf->baro.p   = compensate_pressure((uint32_t)buf->baro.p);
  buf->baro.alt = compute_relative_altitude(buf->baro.p);
  
  buf->accl.x *= MG;
  buf->accl.y *= MG;
  buf->accl.z *= MG;
}

/// Writes one raw sample to the evaluation ring buffer.
void evaluation_put(const struct measurement *buf)
{
  static fu8 idx = 0;

  fu8 i = idx;
  
  payload[i] = *buf;

  i = (i + 1) & RING_MASK;
  fu8 cons = load(&mask, Acq) >> 8;

  if (i != cons) {
    fetch_add(&mask, 1, Rel);
    idx = i;
  }
}


/* ------ Local helpers ------ */

/// Fetches one value from the ring buffer.
/// Returns the amount of new entries added since last call.
static inline fu8
fetch(struct measurement *buf)
{
  static fu8 idx = 0;

  fu16 i = idx;

  /* Fetch and clear new entries counter
   * and store current index */
  fu8 n = (fu8) swap(&mask, i << 8, AcqRel);
  
  if (!n) goto finish;

  i = (i + n - 1) & RING_MASK;
  idx = i;

  *buf = payload[i];

finish:
  /* Always clear busy index (relaxed) */
  fetch_or(&mask, 0xFF00u, Rlx);
  return n;
}

/// Validates one raw measurement against sanity bounds.
static inline fu8
validate(const struct measurement *raw)
{
  enum command st = RAW_DATA;

  if (raw->baro.alt > MAX_ALT || raw->baro.alt < MIN_ALT)
    st += RAW_BAD_ALT;

  if (raw->accl.x > MAX_VAX || raw->accl.x < MIN_VAX)
    st += RAW_BAD_ACC_X;

  if (raw->accl.y > MAX_VAX || raw->accl.y < MIN_VAX)
    st += RAW_BAD_ACC_Y;
  
  if (raw->accl.z > MAX_VAX || raw->accl.z < MIN_VAX)
    st += RAW_BAD_ACC_Z;

  if (raw->gyro.x > MAX_ANG || raw->gyro.x < MIN_ANG)
    st += RAW_BAD_ANG_X;

  if (raw->gyro.y > MAX_ANG || raw->gyro.y < MIN_ANG)
    st += RAW_BAD_ANG_Y;

  if (raw->gyro.z > MAX_ANG || raw->gyro.z < MIN_ANG)
    st += RAW_BAD_ANG_Z;

  st = FC_MSG(st);
  /*
   * Even if data has been successfully validated,
   * send 'OK' signal in order to maintain heartbeat.
   */
  tx_queue_send(&shared, &st, TX_NO_WAIT);

  return st;
}

/// Inverse square root function using bithack.
static inline float invsqrtf(float x)
{
  /* Brought right from Quake 3 Arena! */
  union bithack k = {.f = x};
  k.d = 0x5f3759df - (k.d >> 1);

  /* Newton-Raphson alg */
  for (int i = 0; i < NR_ITERATIONS; ++i) {
    k.f *= 1.5f - 0.5f * x * k.f * k.f;
  }

  return k.f;
}


/* ------ Data evaluation ------ */

/// Cautiously examine altitude changes
/// in order to make an immediate decision.
static inline void
evaluate_altitude(const struct state_vec *vec, fu8 last)
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
detect_launch(const struct state_vec *vec, fu8 last)
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
detect_ascent(const struct state_vec *restrict vec,
              fu8 *restrict sampl, fu8 last)
{
  if (vec[last].v.z > vec[!last].v.z &&
      vec[last].p.z > vec[!last].p.z)
  {
    ++*sampl;
    if (*sampl >= MIN_SAMP_ASCENT)
    {
      flight = ASCENT;
      *sampl = 0;
      log_msg("FC:EVAL: launch confirmed", 26);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && *sampl > 0)
  {
    *sampl = 0;
    enum command cmd = NOT_LAUNCH;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Monitors if minimum threshold for velocity and
/// maximum threshold for acceleration were passed.
/// Checks for height increase and velocity decrease
/// consistency.
static inline void
detect_burnout(const struct state_vec *restrict vec,
               fu8 *restrict sampl, fu8 last)
{
  if (vec[last].v.z >= BURNOUT_MIN_VEL &&
      vec[last].a.z <= BURNOUT_MAX_VAX &&
      vec[last].p.z > vec[last].p.z &&
      vec[last].v.z < vec[!last].v.z)
  {
    ++*sampl;
    if (*sampl >= MIN_SAMP_BURNOUT)
    {
      flight = BURNOUT;
      *sampl = 0;
      log_msg("FC:EVAL: watching for apogee", 29);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && *sampl > 0)
  {
    *sampl = 0;
    enum command cmd = NOT_BURNOUT;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Initially monitors for continuing burnout and
/// for velocity to pass the minimum threshold.
static inline void
detect_apogee(const struct state_vec *vec, fu8 last)
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
detect_descent(const struct state_vec *restrict vec,
               fu8 *restrict sampl, fu8 last)
{
  if (vec[last].p.z < vec[!last].p.z &&
      vec[last].v.z > vec[!last].v.z)
  {
    ++*sampl;
    if (*sampl >= MIN_SAMP_DESCENT)
    {
      flight = DESCENT;
      *sampl = 0;
      co2_high();
      log_msg("FC:EVAL: fired pyro, descending", 32);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && *sampl > 0)
  {
    *sampl = 0;
    enum command cmd = NOT_DESCENT;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Monitors for falling below a specific altitude,
/// and checks for altitude consistency.
static inline void
detect_reef(const struct state_vec *restrict vec,
            fu8 *restrict sampl, fu8 last)
{
  if (vec[last].p.z <= REEF_TARGET_ALT && 
      vec[last].p.z < vec[!last].p.z)
  {
    ++*sampl;
    if (*sampl>= MIN_SAMP_REEF)
    {
      flight = REEF;
      *sampl = 0;
      reef_low();
      log_msg("FC:EVAL: expanded parachute", 28);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && *sampl > 0)
  {
    *sampl = 0;
    enum command cmd = NOT_REEF;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Monitors all statistical metrics to not deviate
/// beyond allowed tolerance thresholds.
static inline void
detect_landed(const struct state_vec *restrict vec,
              fu8 *restrict sampl, fu8 last)
{
  float dh = vec[last].p.z - vec[!last].p.z;
  float dv = vec[last].v.z - vec[!last].v.z;
  float da = vec[last].a.z - vec[!last].a.z;

  if ((dh <= ALT_TOLER && dh >= -ALT_TOLER) &&
      (dv <= VEL_TOLER && dv >= -VEL_TOLER) &&
      (da <= VAX_TOLER && da >= -VAX_TOLER))
  {
    ++*sampl;
    if (*sampl >= MIN_SAMP_LANDED)
    {
      flight = LANDED;
      log_msg("FC:EVAL: rocket landed", 23);
    }
  }
  else if (mode & CONSECUTIVE_SAMP && *sampl > 0)
  {
    sampl = 0;
    enum command cmd = NOT_LANDED;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


/* ------ Unscented Kalman Filter ------ */

/// Transforms state vector into sensor measurement.
static inline void
measurement(const struct state_vec *restrict vec,
            struct measurement *restrict out)
{
  const float ag = vec->a.z + GRAVITY_SI;
  const float qq2 = vec->qv.q2 * vec->qv.q2;
  const float qq3 = vec->qv.q3 * vec->qv.q3;
  const float qq4 = vec->qv.q4 * vec->qv.q4;

  const float q12 = vec->qv.q1 * vec->qv.q2;
  const float q13 = vec->qv.q1 * vec->qv.q3;
  const float q14 = vec->qv.q1 * vec->qv.q4;
  const float q23 = vec->qv.q2 * vec->qv.q3;
  const float q24 = vec->qv.q2 * vec->qv.q4;
  const float q34 = vec->qv.q3 * vec->qv.q4;
  /* 
   * DCM body->nav from quaternion (scalar-first w,x,y,z)
   * Manually transposed (as used). Each value used once
   * => array is not used to allow for compile-time folding
   */
  const float r11 = 1.0f - (2.0f * (qq3 + qq4));
  const float r12 = 2.0f * (q23 + q14);
  const float r13 = 2.0f * (q24 - q13);
  const float r21 = 2.0f * (q23 - q14);
  const float r22 = 1.0f - (2.0f * (qq2 + qq4));
  const float r23 = 2.0f * (q34 + q12);
  const float r31 = 2.0f * (q24 + q13);
  const float r32 = 2.0f * (q34 - q12);
  const float r33 = 1.0f - (2.0f * (qq2 + qq3));

  out->accl.x = (r11 * vec->a.x) + (r12 * vec->a.y) + (r13 * ag);
  out->accl.y = (r21 * vec->a.x) + (r22 * vec->a.y) + (r23 * ag);
  out->accl.z = (r31 * vec->a.x) + (r32 * vec->a.y) + (r33 * ag);

  out->gyro = vec->w;
  out->baro.alt = vec->p.z;
}

/// Transforms input vector into next-sample prediction.
static inline void predict(struct state_vec *vec)
{
  /* Needed for divisibility test by a power of 2
   * overflow is OK */
  static fu8 iter = 0;

  const float dt = FSEC(timer_fetch_update(Predict));

  const float fact = 0.5f * dt;
  const float dvx = dt * vec->a.x;
  const float dvy = dt * vec->a.y;
  const float dvz = dt * vec->a.z;
  const float vtx = dt * vec->v.x;
  const float vty = dt * vec->v.y;
  const float vtz = dt * vec->v.z;

  const struct quaternion old = vec->qv;

  vec->v.x += dvx;
  vec->v.y += dvy;
  vec->v.z += dvz;
  
  vec->p.x += vtx + (fact * dvx);
  vec->p.y += vty + (fact * dvy);
  vec->p.z += vtz + (fact * dvz);

  vec->qv.q1 -= fact * (vec->w.x * old.q2 + \
                        vec->w.y * old.q3 + \
                        vec->w.z * old.q4);
  vec->qv.q2 += fact * (vec->w.x * old.q1 + \
                        vec->w.z * old.q3 - \
                        vec->w.y * old.q4);
  vec->qv.q3 += fact * (vec->w.y * old.q1 - \
                        vec->w.z * old.q2 + \
                        vec->w.x * old.q4);
  vec->qv.q4 += fact * (vec->w.z * old.q1 + \
                        vec->w.y * old.q2 - \
                        vec->w.x * old.q3);

  if ( mode & RENORM_QUATERN_1                  ||
      (mode & RENORM_QUATERN_2 && !(iter & 1u)) ||
      (mode & RENORM_QUATERN_4 && !(iter & 3u)) ||
      (mode & RENORM_QUATERN_8 && !(iter & 7u)))
  {
    const float expr = vec->qv.q1 * vec->qv.q1 + \
                       vec->qv.q2 * vec->qv.q2 + \
                       vec->qv.q3 * vec->qv.q3 + \
                       vec->qv.q4 * vec->qv.q4;
    
    if (expr < TLOWER_1 || expr > TUPPER_1) {
      const float norm = invsqrtf(expr);
      vec->qv.q1 *= norm;
      vec->qv.q2 *= norm;
      vec->qv.q3 *= norm;
      vec->qv.q4 *= norm;
    }
  }

  ++iter;
}

/* Static covariance matrices */

static float state_cov[16][16] = {0}; // P_0
static float noise_cov[16][16] = {0}; // Q
static float meas_cov [16][7]  = {0}; // R

/// Ascent Kalman Filter (ascentKF.m)
static inline void
ascentKF(struct state_vec *vec, const struct measurement *meas)
{
  // TODO
}

/// Descent Kalman Filter (descentKF.m)
static inline void
descentKF(struct state_vec *vec, const struct measurement *meas)
{
  // TODO
}


/* ------ Evaluation Task ------ */

/// High-level overview of data evaluation cycle.
void evaluation_entry(ULONG input)
{
  (void) input;

  log_msg("FC:EVAL: received launch signal", 32);

  /* Signal distribution task to enter main cycle */
  fetch_or(&config, ENTER_DIST_CYCLE, Rel);

  struct state_vec vec[2] = {0};
  struct measurement raw = {0};

  fu8 last = 0;
  fu8 samp = 0;
  flight = IDLE;
  
  /* Begin counting down elapsed time for 
  * KF kinematic equations (predict function) */
  timer_update(Predict);  
  
  task_loop (mode & ABORT_EVALUATION)
  {
    /* See if there are any changes in config */
    mode = load(&config, Acq);

    if (!fetch(&raw) || validate(&raw) > RAW_DATA)
    {
      tx_thread_sleep(EVAL_SLEEP_NO_DATA);
      continue;
    }

    last = !last;

    flight < APOGEE ? ascentKF (&vec[last], &raw) :
                      descentKF(&vec[last], &raw) ;

    /* See if there are any changes in config */
    tx_thread_sleep(EVAL_SLEEP_RT_CONF);
    mode = load(&config, Acq);

    if (mode & FORCE_ALT_CHECKS) {
      evaluate_altitude(vec, last);
    }

    switch (flight) {
      case IDLE:
        detect_launch(vec, last);
        break;
      case LAUNCH:
        detect_ascent(vec, &samp, last);
        break;
      case ASCENT:
        detect_burnout(vec, &samp, last);
        break;
      case BURNOUT:
        detect_apogee(vec, last);
        break;
      case APOGEE:
        detect_descent(vec, &samp, last);
        break;
      case DESCENT:
        detect_reef(vec, &samp, last);
        break;
      case REEF:
        detect_landed(vec, &samp, last);
        break;
      default: break;
    }

    /* Cycle Distribution and Telemetry tasks */
    tx_thread_relinquish();
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