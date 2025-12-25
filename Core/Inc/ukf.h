/*
 * UKF definitions and API
 * combined in one header.
 */

#define G_MPS2 9.80665f
#define TOLERANCE 1e-3f
#define TLOWER_1 (1.0f - TOLERANCE)
#define TUPPER_1 (1.0f + TOLERANCE)

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

/* Globals */