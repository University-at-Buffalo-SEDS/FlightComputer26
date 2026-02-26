/*
 * Kalman Filters
 *
 * This file includes implementations of:
 *   - Unscented (ascent) Kalman filter;
 *	 - Regular (descent) Kalman filter;
 *   - Covariance initialization functions;
 *	 - Context and type-specifc math functions.
 *
 * The entirety of logic in this file is executed within
 * the context of either Distribution Evaluation task,
 * which invoke parts of one of two filters based on the
 * flight state and global run time configuration. All
 * invokationsare performed synchronously.
 *
 * This file does not implement linear algebra functions,
 * and instead casts structs to column vectors and passes
 * references to their wrappers to the CMSIS library.
 *
 * Both KF implementations share the same buffers, the size
 * of which equals to the largest demanded size among the
 * two filters. When the filters are switched, the buffers
 * are cleared and set to the default values expected by
 * the newly selected filter.
 */

#include "evaluation.h"
#include "recovery.h"
#include "platform.h"
#include "kalman.h"
#include "dma.h"


/* Number of iterations - 1 for quaternion matrix
 * renormalization. Set by Recovery task. */
volatile fu8 renorm_step_mask = RENORM_STEP;


/* ------ Local math functions ------ */

/*
 * Inverse square root function using bithack.
 */
static inline constexpr float invsqrtf(float x)
{
  /* Brought right from Quake 3 Arena! */
  union bithack k = {.f = x};
  k.d = 0x5f3759df - (k.d >> 1);

  /* Newton-Raphson alg */
  for (fu8 i = 0; i < NR_ITERATIONS; ++i) {
    k.f *= 1.5f - 0.5f * x * k.f * k.f;
  }

  return k.f;
}

/* ------ Local math functions ------ */


/* ------ Shared static buffers ------ */

static float P[L][L] = {0};
static float Q[L][L] = {0};
static float A[L][L] = {0};
static float R[M][M] = {0};
static float H[M][M] = {0};

static matrix state_cov = {0, 0, &P[0][0]};
static matrix prc_noise = {0, 0, &Q[0][0]};
static matrix mx_bucket = {0, 0, &A[0][0]};
static matrix measm_cov = {0, 0, &R[0][0]};
static matrix obsrvance = {0, 0, &H[0][0]};

static const float W_a[W_DIM] = {[0] = W_0_a, [1 ... W_l] = W_l_a};
static const float W_c[W_DIM] = {[0] = W_0_c, [1 ... W_l] = W_l_c};

static matrix vec_w_a = {W_DIM, 1, (float *)(&W_a[0])};
static matrix vec_w_c = {W_DIM, 1, (float *)(&W_c[0])};

/* ------ Shared static buffers ------ */


/* ------ Descent Kalman filter ------ */

/*
 * Sets descent filter values in shared buffers.
 * Called by Evaluation task when APOGEE state is reached.
 */
void initialize_descent(void)
{
  sv_size_bytes = DESC_STAT * sizeof(float);

  memset(P, 0, sizeof P);
  memset(Q, 0, sizeof Q);
  memset(A, 0, sizeof A);
  memset(R, 0, sizeof R);
  memset(H, 0, sizeof H);

  for (fu8 i = 0; i < DESC_STAT; ++i) {
    Q[i][i] = TOLERANCE;
    A[i][i] = 1.0f;
  }

  for (fu8 i = 0; i < DESC_MEAS - 1; ++i) {
    H[i][i] = 1.0f;
    R[i][i] = 10.0f;
  }

  H[DESC_MEAS - 1][DESC_MEAS - 2] = 1.0f;
  R[DESC_MEAS - 1][DESC_MEAS - 1] = 100.0f;

  state_cov.numRows = state_cov.numCols = DESC_STAT;
  prc_noise.numRows = prc_noise.numCols = DESC_STAT;
  mx_bucket.numCols = mx_bucket.numRows = DESC_STAT;
  measm_cov.numCols = measm_cov.numRows = DESC_MEAS;
  obsrvance.numRows = obsrvance.numCols = DESC_MEAS;

  /* Disable interrupts from IMU */
  __NVIC_DisableIRQ(Gyro_EXTI_1);
  __NVIC_DisableIRQ(Gyro_EXTI_2);
  __NVIC_DisableIRQ(Accl_EXTI_1);
  __NVIC_DisableIRQ(Accl_EXTI_2);

  timer_update(DescentKF);
  fetch_and(&config, ~option(Using_Ascent_KF), Rel);
}

/*
 * descentKF.m
 */
void descentKF(const struct descent *z)
{
  matrix state = {DESC_STAT, 1, (float *)&sv[sh.idx]};
  matrix measm = {DESC_MEAS, 1, (float *)z};

	const float dt = fsec(timer_exchange(DescentKF));

	A[0][APEX_A] = A[1][APEX_A + 1] = A[2][APEX_A + 2] = dt;

	// TODO
}

/* ------ Descent Kalman filter ------ */


/* ------ Ascent Kalman filter ------ */

/*
 * Sets ascent filter values in shared buffers.
 * Called during boot by Evaluation task.
 */
void initialize_ascent(void) 
{
  sv_size_bytes = ASC_STAT * sizeof(float);

  memset(P, 0, sizeof P);
  memset(Q, 0, sizeof Q);
  memset(A, 0, sizeof A);
  memset(R, 0, sizeof R);
  memset(H, 0, sizeof H);

  for (fu8 k = 0; k < 3; ++k) {
    Q[k][k] = 1e-4f;
    Q[k + 3][k + 3] = 1e-2f;
    Q[k + 6][k + 6] = 1e-2f;

    P[k][k] = 100.0f;
    P[k + 3][k + 3] = 25.0f;
    P[k + 6][k + 6] = 1.0f;
    P[k + 13][k + 13] = 1e-2f;
  }

  for (fu8 k = 9; k < 9 + 4; ++k) {
    Q[k][k] = TOLERANCE * TOLERANCE;
    P[k][k] = TOLERANCE;
  }

  Q[13][13] = Q[14][14] = 1e-4f;
  Q[15][15] = 1e-10f;

  R[0][0] = R[1][1] = SIGMA_GYRO * SIGMA_GYRO;
  R[2][2] = SIGMA_GYRO_Z * SIGMA_GYRO_Z;
  R[3][3] = R[4][4] = R[5][5] = SIGMA_ACC * SIGMA_ACC;
  R[6][6] = SIGMA_ALT * SIGMA_ALT;

  state_cov.numRows = state_cov.numCols = ASC_STAT;
  prc_noise.numRows = prc_noise.numCols = ASC_STAT;
  mx_bucket.numCols = mx_bucket.numRows = ASC_STAT;
  measm_cov.numCols = measm_cov.numRows = ASC_MEAS;
  obsrvance.numRows = obsrvance.numCols = ASC_MEAS;

  /* This deduction is due Using_Ascent_KF being in DEFAULT_OPTIONS */
  if (!(load(&config, Acq) & option(Using_Ascent_KF)))
  {
    /* If it somehow happened that we initialize Ascent KF mid-flight,
     * ask non-preemtive recovery to reinitialize IMU with interrupts. */
    enum message cmd = fc_mask(Enable_IMU);
    tx_queue_send(&shared, &cmd, TX_WAIT_FOREVER);
  }

  timer_update(AscentKF);
  fetch_or(&config, option(Using_Ascent_KF), Rel);
}

/*
 * Transforms input vector into next-sample prediction.
 */
void predict(const float dt)
{
  static fu8 iteration = 0;

  const float fact = 0.5f * dt;
  const float dvx = dt * sv[sh.idx].a.x;
  const float dvy = dt * sv[sh.idx].a.y;
  const float dvz = dt * sv[sh.idx].a.z;
  const float vtx = dt * sv[sh.idx].v.x;
  const float vty = dt * sv[sh.idx].v.y;
  const float vtz = dt * sv[sh.idx].v.z;

  const struct quaternion old = sv[sh.idx].qv;

  sv[sh.idx].v.x += dvx;
  sv[sh.idx].v.y += dvy;
  sv[sh.idx].v.z += dvz;
  
  sv[sh.idx].p.x += vtx + (fact * dvx);
  sv[sh.idx].p.y += vty + (fact * dvy);
  sv[sh.idx].p.z += vtz + (fact * dvz);

  sv[sh.idx].qv.q1 -= fact * (sv[sh.idx].w.x * old.q2 + \
                              sv[sh.idx].w.y * old.q3 + \
                              sv[sh.idx].w.z * old.q4);
  sv[sh.idx].qv.q2 += fact * (sv[sh.idx].w.x * old.q1 + \
                              sv[sh.idx].w.z * old.q3 - \
                              sv[sh.idx].w.y * old.q4);
  sv[sh.idx].qv.q3 += fact * (sv[sh.idx].w.y * old.q1 - \
                              sv[sh.idx].w.z * old.q2 + \
                              sv[sh.idx].w.x * old.q4);
  sv[sh.idx].qv.q4 += fact * (sv[sh.idx].w.z * old.q1 + \
                              sv[sh.idx].w.y * old.q2 - \
                              sv[sh.idx].w.x * old.q3);

  if (!(iteration & renorm_step_mask))
  {
    const float expr = sv[sh.idx].qv.q1 * sv[sh.idx].qv.q1 + \
                       sv[sh.idx].qv.q2 * sv[sh.idx].qv.q2 + \
                       sv[sh.idx].qv.q3 * sv[sh.idx].qv.q3 + \
                       sv[sh.idx].qv.q4 * sv[sh.idx].qv.q4;
    
    if (expr < sigma_low(1) || expr > sigma_high(1)) {
      const float norm = invsqrtf(expr);
      sv[sh.idx].qv.q1 *= norm;
      sv[sh.idx].qv.q2 *= norm;
      sv[sh.idx].qv.q3 *= norm;
      sv[sh.idx].qv.q4 *= norm;
    }
  }

  ++iteration;
}

/*
 * Transforms state vector into sensor measurement.
 */
static inline void
measurement(const struct state_vec *restrict vec,
            struct measm_z *restrict out)
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

  out->d.axis.accl.x = (r11 * vec->a.x) + (r12 * vec->a.y) + (r13 * ag);
  out->d.axis.accl.y = (r21 * vec->a.x) + (r22 * vec->a.y) + (r23 * ag);
  out->d.axis.accl.z = (r31 * vec->a.x) + (r32 * vec->a.y) + (r33 * ag);

  out->gyro = vec->w;
  out->d.alt = vec->p.z;
}

/*
 * Update portion of the Ascent KF.
 */
void update(const float dt)
{
  return;
}

/*
 * ascentKF.m
 */
void ascentKF(const struct measm_z *z)
{
  chol_lower_triang(&state_cov, &mx_bucket);

  // TODO s and x_j (L x 2L + 1), ops over x_0 and A -> s
  // TODO x_0 becomes x_hat AFTER ^^^^

  // memset(x_0, 0, sizeof *x_0);

  // TODO S_hat becomes H (shared)

  // TODO
}

/* ------ Ascent Kalman filter ------ */