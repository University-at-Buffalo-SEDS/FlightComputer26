/*
 * Kalman filter and math functions
 *
 * This file includes implementations of:
 *   - Unscented (ascent) Kalman filter;
 *	 - Regular (descent) Kalman filter;
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
 * two filters. When the flight state switches to Apogee,
 * the buffers are cleared and set to the default values
 * expected by the descent filter.
 */

#include "evaluation.h"
#include "recovery.h"
#include "platform.h"
#include "kalman.h"
#include "dma.h"

/// Module config bitmask
/// Origin: Evaluation task
extern fu16 mode;


/* ------ Locally used definitions ------ */

#define TOLERANCE 1e-3f
#define FTLOW(k)  ((float)k - TOLERANCE)
#define FTHIGH(k) ((float)k + TOLERANCE)

#define FSEC(ms) ((float)(ms) * 0.001f)

#define NR_ITERATIONS 2

/* For ascent filter */
#define ASC_STAT 16
#define ASC_MEAS 7

/* For descent filter */
#define DESC_STAT 6
#define DESC_MEAS 4
#define DESC_MASK (DESC_STAT - 1)
#define APEX_A    (DESC_MASK - 2)

union bithack {
  float f;
  uint32_t d;
};


/* ------ Math functions ------ */

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

  out->d.accl.x = (r11 * vec->a.x) + (r12 * vec->a.y) + (r13 * ag);
  out->d.accl.y = (r21 * vec->a.x) + (r22 * vec->a.y) + (r23 * ag);
  out->d.accl.z = (r31 * vec->a.x) + (r32 * vec->a.y) + (r33 * ag);

  out->gyro = vec->w;
  out->baro.alt = vec->p.z;
}

/// Transforms input vector into next-sample prediction.
static inline void
predict(struct state_vec *vec)
{
  /* Needed for divisibility test by a power of 2
   * => overflow is OK */
  static fu8 iter = 0;

  const float dt = FSEC(timer_fetch_update(AscentKF));

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
    
    if (expr < FTLOW(1) || expr > FTHIGH(1)) {
      const float norm = invsqrtf(expr);
      vec->qv.q1 *= norm;
      vec->qv.q2 *= norm;
      vec->qv.q3 *= norm;
      vec->qv.q4 *= norm;
    }
  }

  ++iter;
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


#if defined (TELEMETRY_ENABLED) && defined (GPS_AVAILABLE) 

/* ------ Descent Kalman filter ------ */

/// Sets descent filter values in shared buffers.
/// Called by Evaluation task when APOGEE state is reached.
void initialize_descent()
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

/// descentKF.m
void
descentKF(struct state_vec *x_0, struct state_vec *x_f,
          const struct descent *z)
{
  /* Custom wrappers for the shared pools. */
  static const matrix noise = {DESC_STAT, DESC_STAT, &Q[0][0]};
  static const matrix trans = {DESC_STAT, DESC_STAT, &A[0][0]};
  static const matrix obsrv = {DESC_MEAS, DESC_STAT, &H[0][0]};
  static const matrix mscov = {DESC_MEAS, DESC_MEAS, &R[0][0]};

  static matrix state = {DESC_STAT, 1, NULL};
  static matrix measm = {DESC_MEAS, 1, NULL};

  /* Set wrappers to point to given column vectors. */
  state.pData = (float *)x_0;
  measm.pData = (float *)z;

	const float dt = FSEC(timer_fetch_update(DescentKF));

  /* Set marked fields to time elapsed. */
	A[0][APEX_A    ] = dt;
	A[1][APEX_A + 1] = dt;
	A[2][APEX_A + 2] = dt;

	// TODO
}

#endif // TELEMETRY_ENABLED * GPS_AVAILABLE


/* ------ Ascent (unscented) Kalman filter ------ */

/// ascentKF.m
void
ascentKF(struct state_vec *x_0, struct state_vec *x_f,
         const struct measm_z *z)
{
	// TODO
}