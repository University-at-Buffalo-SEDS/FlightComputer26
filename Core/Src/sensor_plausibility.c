/* Core/Src/sensor_plausibility.c */

#include "sensor_plausibility.h"
#include "rud_detect.h"
#include <string.h>

#define WINDOW_MASK ((uint32_t)((1UL << RUD_WINDOW) - 1UL))

static bool finite_f(float v)
{
  union { float f; uint32_t u; } c = { .f = v };
  return (c.u & 0x7f800000u) != 0x7f800000u;
}

void plaus_reset(plaus_state *p)
{
  memset(p, 0, sizeof *p);
}

bool plaus_baro(plaus_state *p, uint32_t t_ms, float alt)
{
  bool vote = !finite_f(alt);
  if (finite_f(alt))
  {
    if (p->have_baro && t_ms != p->last_t)
    {
      const float limit = RUD_PRESSURE_RATE_MPS * (float)(uint32_t)(t_ms - p->last_t) * 0.001f;
      const float step = alt - p->last_alt;
      vote = step * step >= limit * limit;
      if (step == 0.0f) { if (p->stuck_run < PLAUS_BARO_STUCK_STEPS) ++p->stuck_run; }
      else p->stuck_run = 0;
      p->baro_stuck = p->stuck_run >= PLAUS_BARO_STUCK_STEPS;
    }
    p->last_alt = alt;
    p->last_t = t_ms;
    p->have_baro = true;
  }
  p->baro_votes = (p->baro_votes << 1) | (uint32_t)vote;
  const unsigned n = (unsigned)__builtin_popcount((unsigned)(p->baro_votes & WINDOW_MASK));
  if (n >= RUD_PRESSURE_MIN) p->baro_bad = true;
  else if (n <= PLAUS_BARO_CLEAR_MAX) p->baro_bad = false;
  /* A sample is used only if it moved at a physical rate and the barometer
   * as a whole is plausible. */
  return !vote && !p->baro_bad && !p->baro_stuck;
}

void plaus_baro_missing(plaus_state *p)
{
  p->baro_votes = (p->baro_votes << 1) | 1U;
  const unsigned n = (unsigned)__builtin_popcount((unsigned)(p->baro_votes & WINDOW_MASK));
  if (n >= RUD_PRESSURE_MIN) p->baro_bad = true;
}

bool plaus_baro_ok(const plaus_state *p)
{
  return !p->baro_bad && !p->baro_stuck;
}

bool plaus_baro_suspect(const plaus_state *p)
{
  return (p->baro_votes & WINDOW_MASK) != 0U;
}

bool plaus_baro_repeat(const plaus_state *p)
{
  return p->stuck_run != 0U;
}

bool plaus_baro_clean(const plaus_state *p)
{
  return p->have_baro && (p->baro_votes & WINDOW_MASK) == 0U && p->stuck_run == 0U &&
         !p->baro_bad && !p->baro_stuck;
}

void plaus_velocity(plaus_state *p, float ekf_vel, float trend_vel)
{
  const float d = ekf_vel - trend_vel;
  const bool disagree = !finite_f(d) || d > PLAUS_VEL_TOLER_MPS || d < -PLAUS_VEL_TOLER_MPS;
  if (disagree)
  {
    p->vel_good_run = 0;
    if (p->vel_bad_run < PLAUS_VEL_STEPS) ++p->vel_bad_run;
    if (p->vel_bad_run >= PLAUS_VEL_STEPS) p->vel_bad = true;
  }
  else
  {
    p->vel_bad_run = 0;
    if (p->vel_good_run < PLAUS_VEL_STEPS) ++p->vel_good_run;
    if (p->vel_good_run >= PLAUS_VEL_STEPS) p->vel_bad = false;
  }
}

bool plaus_velocity_ok(const plaus_state *p)
{
  return !p->vel_bad;
}

void plaus_imu(plaus_state *p, const float accel_gyro[6])
{
  if (p->have_imu && memcmp(p->last_imu, accel_gyro, sizeof p->last_imu) == 0)
  {
    if (p->imu_stuck_run < PLAUS_IMU_STUCK_STEPS) ++p->imu_stuck_run;
  }
  else p->imu_stuck_run = 0;
  p->imu_stuck = p->imu_stuck_run >= PLAUS_IMU_STUCK_STEPS;
  memcpy(p->last_imu, accel_gyro, sizeof p->last_imu);
  p->have_imu = true;
}

void plaus_imu_silent(plaus_state *p, bool silent)
{
  p->imu_silent = silent;
}

bool plaus_imu_ok(const plaus_state *p)
{
  return !p->vel_bad && !p->imu_stuck && !p->imu_silent;
}
