/*
 * UKF definitions and API
 * combined in one header.
 */

#define DT_S 0.01f
#define G_MPS2 9.80665f
#define TOLERANCE 1e-3f
#define TLOWER_1 (1.0f - TOLERANCE)
#define TUPPER_1 (1.0f + TOLERANCE)

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

/// Transforms state vector into sensor measurement.
inline void ukf_measurement(const state_vec_t *restrict vec,
                            sensor_meas_t *restrict out);

/// Outputs one next-sample prediction. 
inline void ukf_predict(const state_vec_t *restrict vec,
                        state_vec_t *restrict next);