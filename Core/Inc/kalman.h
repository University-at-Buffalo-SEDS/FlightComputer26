/*
 * Kalman filter configuration and API.
 */

#ifndef KALMAN_H
#define KALMAN_H


/* ------ Public API ------ */

#include "platform.h"
#include "evaluation.h"

/// descentKF.m
void descentKF(struct state_vec *vec, const struct measurement *meas);

/// ascentKF.m
void ascentKF(struct state_vec *vec, const struct measurement *meas);

/// Sets descent filter values in shared buffers.
/// Called by Evaluation task when APOGEE state is reached.
void initialize_descent();

#endif