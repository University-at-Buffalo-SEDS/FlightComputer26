/*
 * Unscented Kalman Filter thread and logic.
 * Target scenario is one physical thread and scheduler.
 */

#include <stdint.h>
#include <stdatomic.h>
#include <math.h>

#include "platform.h"
#include "ukf.h"

TX_THREAD ukf_thread;
ULONG ukf_thread_stack[UKF_THREAD_STACK_SIZE];

/// Public ring and counter for deployment.
sensor_meas_t ring[UKF_RING_SIZE] = {0};
atomic_uint_fast8_t newdata = 0;

/// Last recorded time for each UKF timer user.
static uint32_t time[Time_Users] = {0};

/* General helpers */

/// Reports ms elapsed since last call for each user.
/// Does not handle u32 wrap (flight assumed < 49 days :D).
static inline uint32_t ukf_elapsed_ms(ukf_time_user_e u)
{
  uint32_t prev = time[u];
  time[u] = hal_time_ms();
  return time[u] - prev;
}

/// Updates time for each user to prevent large first returns.
static inline void ukf_initialize_time()
{
  for (ukf_time_user_e u = 0; u < Time_Users; ++u)
    time[u] = hal_time_ms();
}

/* Prediction tools */

/// Transforms state vector into sensor measurement.
static inline void ukf_measurement(const state_vec_t *restrict vec,
                                   sensor_meas_t *restrict out)
{
  const float ag = vec->a.z + G_MPS2;
  const float qq2 = vec->q2 * vec->q2;
  const float qq3 = vec->q3 * vec->q3;
  const float qq4 = vec->q4 * vec->q4;

  const float q12 = vec->q1 * vec->q2;
  const float q13 = vec->q1 * vec->q3;
  const float q14 = vec->q1 * vec->q4;
  const float q23 = vec->q2 * vec->q3;
  const float q24 = vec->q2 * vec->q4;
  const float q34 = vec->q3 * vec->q4;
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

  out->accl.x = (r11 * vec->a.x) + (r12 * vec->a.y) + (r13 * ag);
  out->accl.y = (r21 * vec->a.x) + (r22 * vec->a.y) + (r23 * ag);
  out->accl.z = (r31 * vec->a.x) + (r32 * vec->a.y) + (r33 * ag);

  out->gyro = vec->w;
  out->alt = vec->p.z;
}

/// Outputs one next-sample prediction. 
static inline void ukf_predict(const state_vec_t *restrict vec,
                               state_vec_t *restrict next)
{
  const float dt = FSEC(ukf_elapsed_ms(Predict));

  const float fact = 0.5f * dt;
  const float dvx = dt * vec->a.x;
  const float dvy = dt * vec->a.y;
  const float dvz = dt * vec->a.z;

  next->w.x = vec->w.x;
  next->w.y = vec->w.y;
  next->w.z = vec->w.z;

  next->a.x = vec->a.x;
  next->a.y = vec->a.y;
  next->a.z = vec->a.z;

  next->v.x = dvx + vec->v.x;
  next->v.y = dvy + vec->v.y;
  next->v.z = dvz + vec->v.z;
  
  next->p.x = (fact * dvx) + (dt * vec->v.x) + vec->p.x;
  next->p.y = (fact * dvy) + (dt * vec->v.y) + vec->p.y;
  next->p.z = (fact * dvz) + (dt * vec->v.z) + vec->p.z;

  next->q1 = vec->q1 - (fact * (vec->w.x * vec->q2 + \
                                vec->w.y * vec->q3 + \
                                vec->w.z * vec->q4));
  next->q2 = vec->q2 + (fact * (vec->w.x * vec->q1 + \
                                vec->w.z * vec->q3 - \
                                vec->w.y * vec->q4));
  next->q3 = vec->q3 + (fact * (vec->w.y * vec->q1 - \
                                vec->w.z * vec->q2 + \
                                vec->w.x * vec->q4));
  next->q4 = vec->q4 + (fact * (vec->w.z * vec->q1 + \
                                vec->w.y * vec->q2 - \
                                vec->w.x * vec->q3));

  const float expr = next->q1*next->q1 + next->q2*next->q2 + \
                     next->q3*next->q3 + next->q4*next->q4;

  if (expr < TLOWER_1 || expr > TUPPER_1) {
    const float norm = sqrtf(expr);
    next->q1 /= norm;
    next->q2 /= norm;
    next->q3 /= norm;
    next->q4 /= norm;
  }
}

void ukf_thread_entry(ULONG input)
{
  (void)input;

  log_msg_sync("UKF: starting thread", 21);

  ukf_initialize_time();

  for (;;)
  {
    // switch over value returned by get_rocket_state()
    // to select appropriate filter
  }
}

/// Creates a non-preemptive UKF thread
/// with defined parameters. Called manually.
/// Provides its entry point with a throwaway input.
///
/// Stack size and priority are configurable in FC-Threads.h.
void create_ukf_thread(void)
{
  UINT st = tx_thread_create(&ukf_thread,
                             "UKF Thread",
                             ukf_thread_entry,
                             UKF_THREAD_INPUT,
                             ukf_thread_stack,
                             UKF_THREAD_STACK_SIZE,
                             UKF_THREAD_PRIORITY,
                             /* No preemption */
                             UKF_THREAD_PRIORITY,
                             TX_NO_TIME_SLICE,
                             TX_AUTO_START);

  if (st != TX_SUCCESS) {
    log_die("Failed to create UKF thread: %u", (unsigned)st);
  }
}