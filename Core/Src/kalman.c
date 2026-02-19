/*
 * Kalman Filters
 *
 * This file includes implementations of:
 *   - Unscented (ascent) Kalman filter;
 *	 - Regular (descent) Kalman filter;
 *   - Covariance initialization functions;
 *	 - Context and type-specifc math functions.
 *
 * The entirety of logic in this file is executed
 * within context of the Evaluation task, which
 * invokes one of two filter based on flight state.
 * All invokations are performed synchronously.
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

/// Number of iterations - 1 for quaternion matrix
/// renormalization. Set by Recovery task.
volatile fu8 renorm_step_mask = RENORM_STEP;


/* ------ Local math functions ------ */

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


/* ------ KF helper functions ------ */

/// Transforms state vector into sensor measurement.
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


/// Transforms input vector into next-sample prediction.
static inline void predict(struct state_vec *vec)
{
  static fu8 iteration = 0;

  const float dt = fsec(timer_exchange(AscentKF));

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

  if (!(iteration & renorm_step_mask))
  {
    const float expr = vec->qv.q1 * vec->qv.q1 + \
                       vec->qv.q2 * vec->qv.q2 + \
                       vec->qv.q3 * vec->qv.q3 + \
                       vec->qv.q4 * vec->qv.q4;
    
    if (expr < sigma_low(1) || expr > sigma_high(1)) {
      const float norm = invsqrtf(expr);
      vec->qv.q1 *= norm;
      vec->qv.q2 *= norm;
      vec->qv.q3 *= norm;
      vec->qv.q4 *= norm;
    }
  }

  ++iteration;
}


/* ------ Static buffers ------ */

# if ASC_STAT > DESC_STAT 
#   define L ASC_STAT
# else
#   define L DESC_STAT
# endif

# if ASC_MEAS > DESC_MEAS 
#   define M ASC_MEAS
# else
#   define M DESC_MEAS
# endif

static float P[L][L] = {0};
static float Q[L][L] = {0};
static float A[L][L] = {0};
static float R[M][M] = {0};
static float H[M][M] = {0};


/* ------ Descent Kalman filter ------ */

/// Sets descent filter values in shared buffers.
void initialize_descent(void)
{
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
}


#define DESC_MASK (DESC_STAT - 1)
#define APEX_A    (DESC_MASK - 2)

/// descentKF.m
void
descentKF(struct state_vec *x_0, struct state_vec *x_f,
          const struct descent *z)
{
  static matrix stcov = {DESC_STAT, DESC_STAT, &P[0][0]};
  static matrix noise = {DESC_STAT, DESC_STAT, &Q[0][0]};
  static matrix trans = {DESC_STAT, DESC_STAT, &A[0][0]};
  static matrix obsrv = {DESC_MEAS, DESC_STAT, &H[0][0]};
  static matrix mscov = {DESC_MEAS, DESC_MEAS, &R[0][0]};

  static matrix state = {DESC_STAT, 1, NULL};
  static matrix measm = {DESC_MEAS, 1, NULL};

  state.pData = (float *)x_0;
  measm.pData = (float *)z;

	const float dt = fsec(timer_exchange(DescentKF));

	A[0][APEX_A] = A[1][APEX_A + 1] = A[2][APEX_A + 2] = dt;

	// TODO
}


/* ------ Ascent (unscented) Kalman filter ------ */


#define SIGMA_GYRO 0.1f
#define SIGMA_GYRO_Z 1e-6f
#define SIGMA_ACC 0.1f
#define SIGMA_ALT 10.0f

/// Sets ascent filter values in shared buffers.
void initialize_ascent(void) 
{
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
}


#define ALPHA 1.0f
#define BETA  2.0f
#define KAPPA (1.5f * L)

#define W_DIM (int)(2*L + 1)
#define W_l   (W_DIM - 1)

#define W_0_a (float)((ALPHA*ALPHA*KAPPA - L) / (ALPHA*ALPHA*KAPPA))
#define W_l_a (float)(1.0f / (2.0f*ALPHA*ALPHA*KAPPA))
#define W_0_c (float)(W_0_a + 1.0f - ALPHA*ALPHA + BETA)
#define W_l_c W_l_a

/// ascentKF.m
void
ascentKF(struct state_vec *x_0, struct state_vec *x_f,
         const struct measm_z *z)
{
	static matrix stcov = {ASC_STAT, ASC_STAT, &P[0][0]};
  static matrix noise = {ASC_STAT, ASC_STAT, &Q[0][0]};
  static matrix trans = {ASC_STAT, ASC_STAT, &A[0][0]};
  static matrix obsrv = {ASC_MEAS, ASC_STAT, &H[0][0]};
  static matrix mscov = {ASC_MEAS, ASC_MEAS, &R[0][0]};

  static const float W_a[W_DIM] = {[0] = W_0_a, [1 ... W_l] = W_l_a};
  static const float W_c[W_DIM] = {[0] = W_0_c, [1 ... W_l] = W_l_c};

  static matrix colwa = {W_DIM, 1, (float *)(&W_a[0])};
  static matrix colwc = {W_DIM, 1, (float *)(&W_c[0])};
  
  /* Outputs lower triangular matrix */
  chol(&stcov, &trans);

  // TODO s and x_j (L x 2L + 1), ops over x_0 and A -> s
  // TODO x_0 becomes x_hat AFTER ^^^^

  memset(x_0, 0, sizeof *x_0);

  // TODO S_hat becomes H (shared)

  // TODO
}