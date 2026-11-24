/* Core/Src/rud_detect.c */

#include "rud_detect.h"
#include <string.h>

#define RUD_WINDOW_MASK ((uint32_t)((1UL << RUD_WINDOW) - 1UL))
_Static_assert(RUD_WINDOW < 32U, "one history word");

static bool finite_f(float v)
{
  /* -ffast-math folds isfinite(): test the exponent bits. */
  union { float f; uint32_t u; } c = { .f = v };
  return (c.u & 0x7f800000u) != 0x7f800000u;
}

static unsigned ones(uint32_t v)
{
  return (unsigned)__builtin_popcount((unsigned)(v & RUD_WINDOW_MASK));
}

void rud_reset(rud_state *s)
{
  memset(s, 0, sizeof *s);
}

bool rud_step(rud_state *s, const rud_input *in)
{
  if (!in->armed)
  {
    const bool fired = s->fired;
    rud_reset(s);
    s->fired = fired;
    return false;
  }
  if (s->fired) return false;

  const float wx = in->gyro[0], wy = in->gyro[1];
  const float ax = in->accel[0], ay = in->accel[1];
  const bool tumble = finite_f(wx) && finite_f(wy) &&
                      wx * wx + wy * wy >= RUD_TUMBLE_DPS * RUD_TUMBLE_DPS;
  const bool lateral = finite_f(ax) && finite_f(ay) &&
                       ax * ax + ay * ay >= RUD_LATERAL_MPS2 * RUD_LATERAL_MPS2;

  bool pressure = false;
  if (finite_f(in->baro_alt))
  {
    if (s->have_baro && in->t_ms != s->last_t)
    {
      const float limit = RUD_PRESSURE_RATE_MPS * (float)(uint32_t)(in->t_ms - s->last_t) * 0.001f;
      const float step = in->baro_alt - s->last_alt;
      pressure = step * step >= limit * limit;
    }
    s->last_alt = in->baro_alt;
    s->last_t = in->t_ms;
    s->have_baro = true;
  }

  s->motion = (s->motion << 1) | (uint32_t)(tumble || lateral);
  s->pressure = (s->pressure << 1) | (uint32_t)pressure;
  if (s->steps < RUD_WINDOW) ++s->steps;

  if (s->steps >= RUD_WINDOW && ones(s->motion) >= RUD_MOTION_MIN &&
      ones(s->pressure) >= RUD_PRESSURE_MIN)
  {
    s->fired = true;
    return true;
  }
  return false;
}
