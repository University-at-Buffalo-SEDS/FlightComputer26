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

state_e state = IDLE;


/* ------ Static ------ */

/// Raw data ring.
static sensor_meas_t payload[RING_SIZE] = {0};

/// 0-7:  amount of new entries
/// 8-15: consumer locked index
static atomic_uint_fast16_t mask = 0;

/// Last recorded time for each UKF timer user.
static uint32_t local_time[Time_Users] = {0};


/* ------ Public API ------ */

/// Double-buffering is more memory efficient, but
/// currently it is tricky to implement properly.
void predict_put(const sensor_meas_t *buf)
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
void compensate(sensor_meas_t *buf)
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
uint_fast8_t fetch(sensor_meas_t *buf)
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
static inline int_fast8_t validate(sensor_meas_t *raw)
{
  cmd_e st = CMD_NONE;

  if (raw->baro.alt > MAX_ALT || raw->baro.alt < MIN_ALT)
    st += RAW_BAD_ALT;

  /* Vertical acceleration is a dominating metric */
  if (raw->accl.z > MAX_VAX || raw->accl.z < MIN_VAX)
    st += RAW_BAD_VAX;

  if (raw->gyro.x > MAX_ANG || raw->gyro.x < MIN_ANG)
    st += RAW_BAD_ANG_X;

  if (raw->gyro.y > MAX_ANG || raw->gyro.y < MIN_ANG)
    st += RAW_BAD_ANG_Y;

  if (raw->gyro.z > MAX_ANG || raw->gyro.z < MIN_ANG)
    st += RAW_BAD_ANG_Z;

  if (st < 0)
    tx_queue_send(&shared, &st, TX_NO_WAIT);

  return st;
}

/// Reports ms elapsed since last call for each user.
/// Does not handle u32 wrap (flight assumed < 49 days :D).
static inline uint32_t elapsed_ms(time_user_e u)
{
  uint32_t prev = local_time[u];
  local_time[u] = hal_time_ms();
  return local_time[u] - prev;
}

/// Updates time for each user to prevent large first returns.
static inline void initialize_time()
{
  for (time_user_e u = 0; u < Time_Users; ++u)
  {
    local_time[u] = hal_time_ms();
  }
}

/// ARM fast math is accurate but does not have inverse.
static inline float invsqrtf(float x)
{
  /* Brought right from Quake 3 Arena! */
  bithack_u k = {.f = x};
  k.d = 0x5f3759df - (k.d >> 1);

  /* Newton-Raphson alg */
  for (int i = 0; i < NR_ITERATIONS; ++i) {
    k.f *= 1.5f - 0.5f * x * k.f * k.f;
  }

  return k.f;
}


/* ------ State machine ------ */

/// Monitors if minimum thresholds for velocity and
/// acceleration were exceded.
static inline void detect_launch(const state_vec_t *vec,
                                 uint_fast8_t last)
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
static inline void detect_ascent(const state_vec_t *restrict vec,
                                 uint_fast8_t *restrict samples,
                                 uint_fast8_t last)
{
  if (vec[last].v.z > vec[!last].v.z &&
      vec[last].p.z > vec[!last].p.z)
  {
    ++*samples;
    if (*samples >= MIN_SAMP_ASCENT)
    {
      state = ASCENT;
      *samples = 0;
      log_msg("DEPL: Launch confirmed", 23);
    }
  }
#if CONSECUTIVE_CONFIRMS > 0
  else if (*samples > 0)
  {
    *samples = 0;
    cmd_e cmd = NOT_LAUNCH;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
#endif
}

/// Monitors if minimum threshold for velocity and
/// maximum threshold for acceleration were passed.
/// Checks for height increase and velocity decrease
/// consistency.
static inline void detect_burnout(const state_vec_t *restrict vec,
                                  uint_fast8_t *restrict samples,
                                  uint_fast8_t last)
{
  if (vec[last].v.z >= BURNOUT_MIN_VEL &&
      vec[last].a.z <= BURNOUT_MAX_VAX &&
      vec[last].p.z > vec[last].p.z &&
      vec[last].v.z < vec[!last].v.z)
  {
    ++*samples;
    if (*samples >= MIN_SAMP_BURNOUT)
    {
      state = BURNOUT;
      *samples = 0;
      log_msg("DEPL: Watching for apogee", 26);
    }
  }
#if CONSECUTIVE_CONFIRMS > 0
  else if (*samples > 0)
  {
    *samples = 0;
    cmd_e cmd = NOT_BURNOUT;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
#endif
}

/// Initially monitors for continuing burnout and
/// for velocity to pass the minimum threshold.
static inline void detect_apogee(const state_vec_t *vec,
                                 uint_fast8_t last)
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
static inline void detect_descent(const state_vec_t *restrict vec,
                                  uint_fast8_t *restrict samples,
                                  uint_fast8_t last)
{
  if (vec[last].p.z < vec[!last].p.z &&
      vec[last].v.z > vec[!last].v.z)
  {
    ++*samples;
    if (*samples >= MIN_SAMP_DESCENT)
    {
      state = DESCENT;
      *samples = 0;
      co2_high();
      log_msg("DEPL: Fired pyro, descending", 29);
    }
  }
#if CONSECUTIVE_CONFIRMS > 0
  else if (*samples > 0)
  {
    *samples = 0;
    cmd_e cmd = NOT_DESCENT;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
#endif
}

/// Monitors for falling below a specific altitude,
/// and checks for altitude consistency.
static inline void detect_reef(const state_vec_t *restrict vec,
                               uint_fast8_t *restrict samples,
                               uint_fast8_t last)
{
  if (vec[last].p.z <= REEF_TARGET_ALT && 
      vec[last].p.z < vec[!last].p.z)
  {
    ++*samples;
    if (*samples>= MIN_SAMP_REEF)
    {
      state = REEF;
      *samples = 0;
      reef_low();
      log_msg("DEPL: Expanded parachute", 25);
    }
  }
#if CONSECUTIVE_CONFIRMS > 0
  else if (*samples > 0)
  {
    *samples = 0;
    cmd_e cmd = NOT_REEF;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
#endif
}

/// Monitors all statistical metrics to not deviate
/// beyond allowed tolerance thresholds.
static inline void detect_landed(const state_vec_t *restrict vec,
                                 uint_fast8_t *restrict samples,
                                 uint_fast8_t last)
{
  float dh = vec[last].p.z - vec[!last].p.z;
  float dv = vec[last].v.z - vec[!last].v.z;
  float da = vec[last].a.z - vec[!last].a.z;

  if ((dh <= ALT_TOLER && dh >= -ALT_TOLER) &&
      (dv <= VEL_TOLER && dv >= -VEL_TOLER) &&
      (da <= VAX_TOLER && da >= -VAX_TOLER))
  {
    ++*samples;
    if (*samples >= MIN_SAMP_LANDED)
    {
      state = LANDED;
      log_msg("DEPL: Rocket landed", 20);
    }
  }
#if CONSECUTIVE_CONFIRMS > 0
  else if (*samples > 0)
  {
    samples = 0;
    cmd_e cmd = NOT_LANDED;
    tx_queue_send(&shared, &cmd, TX_NO_WAIT);
  }
#endif
}

/// TODO: 1. State regression
///       2. Discuss bounds in predict.h
///       3. Discuss transition conditions
///       4. Discuss amount of samples
static inline void evaluate(const state_vec_t *vec,
                            uint_fast8_t last)
{
  static uint_fast8_t samples = 0;

  switch (state) {
    case IDLE:
      return detect_launch(vec, last);
    case LAUNCH:
      return detect_ascent(vec, &samples, last);
    case ASCENT:
      return detect_burnout(vec, &samples, last);
    case BURNOUT:
      return detect_apogee(vec, last);
    case APOGEE:
      return detect_descent(vec, &samples, last);
    case DESCENT:
      return detect_reef(vec, &samples, last);
    case REEF:
      return detect_landed(vec, &samples, last);
    default: return;
  }
}


/* ------ Unscented Kalman Filter ------ */

/// Transforms state vector into sensor measurement.
static inline void measurement(const state_vec_t *vec,
                               sensor_meas_t *out)
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
static inline void predict(state_vec_t *vec)
{
  const float dt = FSEC(elapsed_ms(Predict));

  const float fact = 0.5f * dt;
  const float dvx = dt * vec->a.x;
  const float dvy = dt * vec->a.y;
  const float dvz = dt * vec->a.z;
  const float vtx = dt * vec->v.x;
  const float vty = dt * vec->v.y;
  const float vtz = dt * vec->v.z;

  const quaternion_t old = vec->qv;

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

static inline void ascentKF(state_vec_t *vec, // x_0
                            const sensor_meas_t *meas, // z_0
                            float *restrict state_cov, // P_0
                            float *restrict noise_cov, // Q
                            float *restrict meas_cov) // R
{
  // TODO
}

static inline void descentKF(state_vec_t *vec, // x_0
                             const sensor_meas_t *meas, // z_0
                             float *restrict state_cov, // P_0
                             float *restrict noise_cov, // Q
                             float *restrict meas_cov) // R
{
  // TODO
}

/// Runner that holds large static arrays.
static inline void run_ukf(state_vec_t *vec, sensor_meas_t *meas)
{
  /* Covariance matrices */
  static float state_cov[16][16] = {0}; // P_0
  static float noise_cov[16][16] = {0}; // Q
  static float meas_cov [16][7]  = {0}; // R

  if (state < APOGEE) {
    ascentKF(vec, meas, &state_cov[0][0],
             &noise_cov[0][0], &meas_cov[0][0]);
  } else {
    descentKF(vec, meas, &state_cov[0][0],
              &noise_cov[0][0], &meas_cov[0][0]);
  }
}


/* ------ Prediction Task ------ */

/// High-level overview of data evaluation cycle.
void predict_entry(ULONG last)
{
  sensor_meas_t raw;
  state_vec_t vec[2] = {0};

  last = 0;
  initialize_time();

  while (SEDS_ARE_COOL)
  {
    if (!fetch(&raw) || validate(&raw) < 0)
    {
      tx_thread_sleep(PREDICT_SLEEP);
      continue;
    }

    last = !last;
    run_ukf(&vec[last], &raw);

    evaluate(vec, last);
  }
}

/// Creates a non-preemptive UKF thread
/// with defined parameters. Called manually.
/// Provides its entry point with a throwaway input.
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