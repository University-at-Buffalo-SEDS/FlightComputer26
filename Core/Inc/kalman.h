/*
 * Kalman filter configuration and API.
 */

#ifndef KALMAN_H
#define KALMAN_H


/* ------ State and measurement vector sizes ------ */

#define ASC_STAT 16
#define ASC_MEAS 7
#define DESC_STAT 6
#define DESC_MEAS 4


/* ------ Locally used definitions ------ */

#define TOLERANCE 1e-3f

#define sigma_low(k)  ((float)k - TOLERANCE)
#define sigma_high(k) ((float)k + TOLERANCE)

#define FSEC(ms) ((float)(ms) * 0.001f)

#define NR_ITERATIONS 2

union bithack {
  float f;
  uint32_t d;
};


/* ------ Public API ------ */

#include "platform.h"
#include "evaluation.h"

/// descentKF.m
void
descentKF(struct state_vec *x_0, struct state_vec *x_f,
          const struct descent *z);

/// ascentKF.m
void
ascentKF(struct state_vec *x_0, struct state_vec *x_f,
         const struct measm_z *z);

/// Sets descent filter values in shared buffers.
/// Called by Evaluation task when APOGEE state is reached.
void initialize_descent(void);

/// Sets ascent filter values in shared buffers.
/// Called during boot and whenever system falls back to UKF.
void initialize_ascent(void);


#endif // KALMAN_H