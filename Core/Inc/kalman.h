/*
 * Kalman filter configuration and API.
 */

#ifndef KALMAN_H
#define KALMAN_H


/* ------ Public API ------ */

#include "platform.h"
#include "evaluation.h"

/// descentKF.m
void
descentKF(struct state_vec *x_0, struct state_vec *x_f,
          const struct measurement *z);

/// ascentKF.m
void
ascentKF(struct state_vec *x_0, struct state_vec *x_f,
         const struct measurement *z);

/// Sets descent filter values in shared buffers.
/// Called by Evaluation task when APOGEE state is reached.
void initialize_descent();


#endif // KALMAN_H