/*
 * Data predicition and evaluation task.
 * Target scenario is one physical thread and scheduler.
 */

#include <stdint.h>
#include <stdatomic.h>

#include "dsp/fast_math_functions.h"

#include "predict.h"
#include "platform.h"
#include "recovery.h"
#include "dma.h"

TX_THREAD predict_task;
ULONG predict_stack[PREDICT_STACK_ULONG];

/// Current flight state
enum state state = IDLE;


/* ------ Static ------ */

/// Raw data ring.
static struct measurement payload[RING_SIZE] = {0};

/// 0-7:  amount of new entries
/// 8-15: consumer locked index
static atomic_uint_fast16_t mask = 0;

/// Local module config bitmask
static struct op_mode mode = {0};


/* ------ Public API ------ */

/// Double-buffering is more memory efficient, but
/// currently it is tricky to implement properly.
void predict_put(const struct measurement *buf)
{
  static uint_fast8_t idx = 0;

  uint_fast8_t i = idx;
  
  payload[i] = *buf;
  i = (i + 1) & RING_MASK;

  uint_fast8_t cons;
  cons = atomic_load_explicit(&mask, memory_order_acquire) >> 8;

  if (i != cons) {
    atomic_fetch_add_explicit(&mask, 1, memory_order_release);
    idx = i;
  }
}

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


/* ------ Helpers ------ */

/// Single-fetch version of previously designed
/// multiple-fetch algorithm.
uint_fast8_t fetch(struct measurement *buf)
{
  static uint_fast8_t idx = 0;

  uint_fast8_t n;
  uint_fast16_t i = idx;

  /* Fetch and clear new entries counter and store current index */
  n = (uint_fast8_t)atomic_exchange_explicit(&mask, i << 8,
                                             memory_order_acq_rel);
  if (!n) goto finish;

  i = (i + n - 1) & RING_MASK;
  n = 1;

  *buf = payload[i];

finish:
  /* Always clear busy index */
  atomic_fetch_or_explicit(&mask, 0xFF00u, memory_order_release);
  return n;
}

/// Validates one raw measurement against sanity bounds.
static inline int_fast8_t validate(struct measurement *raw)
{
  enum command st = RAW_DATA;

  if (raw->baro.alt > MAX_ALT || raw->baro.alt < MIN_ALT)
    st += RAW_BAD_ALT;

  /* Vertical acceleration is a dominating metric */
  if (raw->accl.z > MAX_VAX || raw->accl.z < MIN_VAX)
    st += RAW_BAD_VAX;

  /* But gyroscope data is used in whole by UKF */
  if (raw->gyro.x > MAX_ANG || raw->gyro.x < MIN_ANG)
    st += RAW_BAD_ANG_X;

  if (raw->gyro.y > MAX_ANG || raw->gyro.y < MIN_ANG)
    st += RAW_BAD_ANG_Y;

  if (raw->gyro.z > MAX_ANG || raw->gyro.z < MIN_ANG)
    st += RAW_BAD_ANG_Z;

  st = FC_MSG(st);
  tx_queue_send(&shared, &st, TX_NO_WAIT);

  return st;
}

/// ARM fast math is accurate but does not have inverse.
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
evaluate_altitude(const struct state_vec *vec, uint_fast8_t last)
{
  /* If we aren't yet ascending or not descending, return.
   * First condition is to guard against noise while idle. */
  if (state < ASCENT || vec[last].p.z > vec[!last].p.z) {
    /* If we were waiting on confirmation BUT config mandates
     * confirmations to be consecutive, reset the flag */
    if (mode.consecutive_samp && mode.pyro_req_confirm) {
      mode.pyro_req_confirm = 0;
    }

    return;
  }

  if (vec[last].p.z <= REEF_TARGET_ALT)
  {
    /* We somehow missed the moment to expand parachute.
      * Do NOT release control until reef is fired */
    if (mode.safe_expand_reef)
    {
      reef_high();
      log_msg("FC: ! urgently expanding reef", 29);
    }
    else
    {
      /* We didn't even expand parachute to that moment.
      * What is going on!!? Perform full sequence. */ 
      co2_high();
      log_msg("FC: ! urgently firing PYRO->REEF", 34);
      mode.safe_expand_reef = 1;
      tx_thread_sleep(100); // sleep for 1 sec
      reef_high();
    }
  }
  else if (!mode.pyro_req_confirm)
  {
    /* Request one confirmation of decreasing height,
      * then fire PYRO. */
    mode.pyro_req_confirm = 1;
  }
  else
  {
    co2_high();
    mode.safe_expand_reef = 1;
  }
}


/// Monitors if minimum thresholds for velocity and
/// acceleration were exceded.
static inline void
detect_launch(const struct state_vec *vec, uint_fast8_t last)
{
  if (vec[last].v.z >= LAUNCH_MIN_VEL &&
      vec[last].a.z >= LAUNCH_MIN_VAX)
  {
    state = LAUNCH;
    log_msg("DEPL: Launch detected", 22);
    tx_thread_sleep(LAUNCH_CONFIRM_DELAY);
  }
}

/// Monitors height and velocity increase consistency.
static inline void
detect_ascent(const struct state_vec *restrict vec,
              uint_fast8_t *restrict sampl, uint_fast8_t last)
{
  if (vec[last].v.z > vec[!last].v.z &&
      vec[last].p.z > vec[!last].p.z)
  {
    ++*sampl;
    if (*sampl >= MIN_SAMP_ASCENT)
    {
      state = ASCENT;
      *sampl = 0;
      log_msg("DEPL: Launch confirmed", 23);
    }
  }
  else if (mode.consecutive_samp && *sampl > 0)
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
               uint_fast8_t *restrict sampl, uint_fast8_t last)
{
  if (vec[last].v.z >= BURNOUT_MIN_VEL &&
      vec[last].a.z <= BURNOUT_MAX_VAX &&
      vec[last].p.z > vec[last].p.z &&
      vec[last].v.z < vec[!last].v.z)
  {
    ++*sampl;
    if (*sampl >= MIN_SAMP_BURNOUT)
    {
      state = BURNOUT;
      *sampl = 0;
      log_msg("DEPL: Watching for apogee", 26);
    }
  }
  else if (mode.consecutive_samp && *sampl > 0)
  {
    *sampl = 0;
    enum command cmd = NOT_BURNOUT;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}

/// Initially monitors for continuing burnout and
/// for velocity to pass the minimum threshold.
static inline void
detect_apogee(const struct state_vec *vec, uint_fast8_t last)
{
  if (vec[last].v.z <= APOGEE_MAX_VEL &&
      vec[last].v.z < vec[!last].v.z)
  {
    state = APOGEE;
    log_msg("DEPL: Approaching apogee", 25);
    tx_thread_sleep(APOGEE_CONFIRM_DELAY);
  }
}

/// Monitors for decreasing altitude and increasing velocity.
static inline void
detect_descent(const struct state_vec *restrict vec,
               uint_fast8_t *restrict sampl, uint_fast8_t last)
{
  if (vec[last].p.z < vec[!last].p.z &&
      vec[last].v.z > vec[!last].v.z)
  {
    ++*sampl;
    if (*sampl >= MIN_SAMP_DESCENT)
    {
      state = DESCENT;
      *sampl = 0;
      co2_high();
      log_msg("DEPL: Fired pyro, descending", 29);
    }
  }
  else if (mode.consecutive_samp && *sampl > 0)
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
            uint_fast8_t *restrict sampl, uint_fast8_t last)
{
  if (vec[last].p.z <= REEF_TARGET_ALT && 
      vec[last].p.z < vec[!last].p.z)
  {
    ++*sampl;
    if (*sampl>= MIN_SAMP_REEF)
    {
      state = REEF;
      *sampl = 0;
      reef_low();
      log_msg("DEPL: Expanded parachute", 25);
    }
  }
  else if (mode.consecutive_samp && *sampl > 0)
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
              uint_fast8_t *restrict sampl, uint_fast8_t last)
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
      state = LANDED;
      log_msg("DEPL: Rocket landed", 20);
    }
  }
  else if (mode.consecutive_samp && *sampl > 0)
  {
    sampl = 0;
    enum command cmd = NOT_LANDED;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
}


/* ------ Unscented Kalman Filter ------ */

/// Transforms state vector into sensor measurement.
static inline void
measurement(const struct state_vec *vec, struct measurement *out)
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


/* ------ Prediction Task ------ */

/// Evaluates run time config in order of decreasing severity
static inline enum global_config
runtime_checks(const struct state_vec *vec,
               uint_fast8_t last, uint_least32_t conf)
{
  if (conf & ABORT_PREDICTION) {
    return ABORT_PREDICTION;
  }
  
  mode.safe_expand_reef = (conf & SAFE_EXPAND_REEF) ? 1 : 0;
  mode.consecutive_samp = (conf & CONSECUTIVE_SAMP) ? 1 : 0;

  if (conf & FORCE_ALT_CHECKS) {
    evaluate_altitude(vec, last);
  }

  return CHECKS_COMPLETE;
}

/// High-level overview of data evaluation cycle.
void predict_entry(ULONG last)
{
  struct state_vec vec[2] = {0};
  struct measurement raw = {0};
  uint_fast8_t sampl = 0;

  last = 0;

  while (SEDS_ARE_COOL)
  {
    if (!fetch(&raw) || validate(&raw) > RAW_DATA)
    {
      tx_thread_sleep(PREDICT_SLEEP);
      continue;
    }

    last = !last;

    state < APOGEE ? ascentKF(&vec[last], &raw) :
                     descentKF(&vec[last], &raw);

    uint_least32_t conf;
    conf = atomic_load_explicit(&config, memory_order_acquire);

    if (runtime_checks(vec, last, conf) == ABORT_PREDICTION) {
      /* Read abort flag in config, shutdown thread */
      return;
    }

    switch (state) {
      case IDLE:
        detect_launch(vec, last);
        break;
      case LAUNCH:
        detect_ascent(vec, &sampl, last);
        break;
      case ASCENT:
        detect_burnout(vec, &sampl, last);
        break;
      case BURNOUT:
        detect_apogee(vec, last);
        break;
      case APOGEE:
        detect_descent(vec, &sampl, last);
        break;
      case DESCENT:
        detect_reef(vec, &sampl, last);
        break;
      case REEF:
        detect_landed(vec, &sampl, last);
        break;
      case LANDED: break;
    }
  }
}

/// Creates a non-preemptive UKF task
/// with defined parameters. Called manually.
///
/// Stack size and priority are configurable in FC-Threads.h.
void create_predict_task(void)
{
  UINT st = tx_thread_create(&predict_task,
                             "Prediction Task",
                             predict_entry,
                             PREDICT_INPUT,
                             predict_stack,
                             PREDICT_STACK_BYTES,
                             PREDICT_PRIORITY,
                             /* No preemption */
                             PREDICT_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("Failed to create Prediction Task: %u", (unsigned)st);
  }
}