/*
 * Functions used by filter invokations.
 */

#include <math.h>
#include "ukf-shared.h"

/// Transforms state vector into sensor measurement.
inline void ukf_measurement(const state_vec_t *restrict vec,
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
inline void ukf_predict(const state_vec_t *restrict vec,
                        state_vec_t *restrict next)
{
  /* Here, we need to obtain time elapsed. For now, use generic
   * DT, but proper timer should be accessible due to the
   * presence of scheduler and ISR. */

  const float fact = 0.5f * DT_S;
  const float dvx = DT_S * vec->a.x;
  const float dvy = DT_S * vec->a.y;
  const float dvz = DT_S * vec->a.z;

  next->w.x = vec->w.x;
  next->w.y = vec->w.y;
  next->w.z = vec->w.z;

  next->a.x = vec->a.x;
  next->a.y = vec->a.y;
  next->a.z = vec->a.z;

  next->v.x = dvx + vec->v.x;
  next->v.y = dvy + vec->v.y;
  next->v.z = dvz + vec->v.z;
  
  next->p.x = (fact * dvx) + (DT_S * vec->v.x) + vec->p.x;
  next->p.y = (fact * dvy) + (DT_S * vec->v.y) + vec->p.y;
  next->p.z = (fact * dvz) + (DT_S * vec->v.z) + vec->p.z;

  next->q1 = vec->q1 - fact * (vec->w.x * vec->q2 + \
                               vec->w.y * vec->q3 + \
                               vec->w.z * vec->q4);
  next->q2 = vec->q2 + fact * (vec->w.x * vec->q1 + \
                               vec->w.z * vec->q3 - \
                               vec->w.y * vec->q4);
  next->q3 = vec->q3 + fact * (vec->w.y * vec->q1 - \
                               vec->w.z * vec->q2 + \
                               vec->w.x * vec->q4);
  next->q4 = vec->q4 + fact * (vec->w.z * vec->q1 + \
                               vec->w.y * vec->q2 - \
                               vec->w.x * vec->q3);

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