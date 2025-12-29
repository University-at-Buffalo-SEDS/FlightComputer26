/*
 * UKF definitions and API
 * combined in one header.
 */

#ifndef UKF_H
#define UKF_H

#define GRAVITY_SI 9.80665f
#define TOLERANCE 1e-3f
#define TLOWER_1 (1.0f - TOLERANCE)
#define TUPPER_1 (1.0f + TOLERANCE)

#define UKF_RING_SIZE 16
#define UKF_RING_MASK (UKF_RING_SIZE - 1)

#define FSEC(ms) ((float)(ms) / 1000.0f)

typedef enum {
  Predict,

  Time_Users
} ukf_time_user_e;

typedef struct { float x, y, z; } coords_t;

typedef struct {
  coords_t p, v, a, w;
  float q1, q2, q3, q4;
} state_vec_t;

typedef struct {
  coords_t gyro, accl;
  float alt;
} sensor_meas_t;

#endif // UKF_H