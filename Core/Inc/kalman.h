/*
 * Kalman filter configuration and API.
 */

#ifndef KALMAN_H
#define KALMAN_H

#include "platform.h"
#include "evaluation.h"

extern volatile fu8 renorm_step_mask;


/* ------ Matrix properties ------ */

#define ASC_STAT 16
#define ASC_MEAS 7
#define DESC_STAT 6
#define DESC_MEAS 4

/* ------ Matrix properties ------ */


/* ------ Locally used definitions ------ */

#define TOLERANCE 1e-3f

#define sigma_low(k)  ((float)k - TOLERANCE)
#define sigma_high(k) ((float)k + TOLERANCE)

#define fsec(ms) ((float)(ms) * 0.001f)

#define NR_ITERATIONS 2

union bithack {
  float f;
  uint32_t d;
};

/* ------ Locally used definitions ------ */


/* ------ Public API ------ */

/*
 * descentKF.m
 */
void descentKF(struct state_vec *x_0, struct state_vec *x_f,
               const struct descent *z);

/*
 * ascentKF.m
 */
void ascentKF(struct state_vec *x_0, struct state_vec *x_f,
              const struct measm_z *z);

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

/* ------ Public API ------ */


#endif // KALMAN_H