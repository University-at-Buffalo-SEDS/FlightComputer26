/*
 * Kalman filter configuration and API.
 */

#ifndef KALMAN_H
#define KALMAN_H

#include "platform.h"
#include "evaluation.h"


/* ------ Matrix properties ------ */

#define ASC_STAT 16
#define ASC_MEAS 7
#define DESC_STAT 6
#define DESC_MEAS 4

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

/* ------ Matrix properties ------ */


/* ------ Ascent filter constants ------ */

#define SIGMA_GYRO        0.1f
#define SIGMA_GYRO_Z      1e-6f
#define SIGMA_ACC         0.1f
#define SIGMA_ALT         10.0f

#define ALPHA             1.0f
#define BETA              2.0f
#define KAPPA             (1.5f * L)

#define W_DIM             (int)(2*L + 1)
#define W_l               (W_DIM - 1)

#define W_0_a             (float)((ALPHA*ALPHA*KAPPA - L) \
                                  / (ALPHA*ALPHA*KAPPA))
#define W_l_a             (float)(1.0f / (2.0f*ALPHA*ALPHA*KAPPA))
#define W_0_c             (float)(W_0_a + 1.0f - ALPHA*ALPHA + BETA)
#define W_l_c             W_l_a

/* ------ Ascent filter constants ------ */


/* ------ Descent filter constants ------ */

#define DESC_MASK (DESC_STAT - 1)
#define APEX_A    (DESC_MASK - 2)

/* ------ Descent filter constants ------ */


/* ------ Shared definitions ------ */

#define TOLERANCE 1e-3f

#define sigma_low(k)  ((float)k - TOLERANCE)
#define sigma_high(k) ((float)k + TOLERANCE)

#define fsec(ms) ((float)(ms) * 0.001f)

#define NR_ITERATIONS 2

union bithack {
  float f;
  uint32_t d;
};

/* ------ Shared definitions ------ */


/* ------ Public API ------ */

/*
 * descentKF.m
 */
void descentKF(const struct descent *z);

/*
 * ascentKF.m
 */
void ascentKF(const struct measm_z *z);

/*
 * Sets descent filter values in shared buffers.
 * Called by Evaluation task when APOGEE state is reached.
 */
void initialize_descent(void);

/*
 * Sets ascent filter values in shared buffers.
 * Called during boot by Evaluation task.
 */
void initialize_ascent(void);

/*
 * Transforms input vector into next-sample prediction.
 */
void predict(const float dt);

/*
 * Update portion of the Ascent KF.
 */
void update(const float dt);

/* ------ Public API ------ */


#endif // KALMAN_H