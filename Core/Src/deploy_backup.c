/* Core/Src/deploy_backup.c */

#include "deploy_backup.h"
#include <string.h>

#define DR_SUBSTEP_MS 40U
#define DR_SPAN_MAX_MS 600000U

/* -ffast-math folds isfinite(): test the exponent bits. */
static bool finite_f(float v)
{
  union { float f; uint32_t u; } c = { .f = v };
  return (c.u & 0x7f800000u) != 0x7f800000u;
}

static bool reached(uint32_t t, uint32_t since)
{
  return (int32_t)(t - since) >= 0;
}

void backup_dr_reset(backup_dr *d)
{
  memset(d, 0, sizeof *d);
}

void backup_dr_trust(backup_dr *d, uint32_t t_ms, float alt, float vel, bool measured)
{
  if (!finite_f(alt) || !finite_f(vel)) return;
  if (measured)
  {
    d->clean_alt = alt;
    d->clean_vel = vel;
    d->clean_ms = t_ms;
    d->have_clean = true;
  }
  else if (d->have_clean)
  {
    /* No-drag ballistic flight from the clean state bounds the climb. */
    const float dt = (float)(int32_t)(t_ms - d->clean_ms) * 0.001f;
    const float v0 = d->clean_vel > 0.0f ? d->clean_vel : 0.0f;
    const float vb = v0 - DR_G * dt;
    const float hb = vb > 0.0f ? d->clean_alt + v0 * dt - 0.5f * DR_G * dt * dt
                               : d->clean_alt + v0 * v0 / (2.0f * DR_G);
    if (alt > hb) alt = hb;
    if (vel > (vb > 0.0f ? vb : 0.0f)) vel = vb > 0.0f ? vb : 0.0f;
  }
  d->ref_alt = alt;
  d->ref_vel = vel;
  d->ref_ms = t_ms;
  d->have_ref = true;
  d->active = false;
}


void backup_dr_chutes(backup_dr *d, uint32_t t_ms, bool drogue_out, bool main_out)
{
  if (drogue_out && !d->drogue) { d->drogue = true; d->drogue_ms = t_ms; }
  if (main_out && !d->main) { d->main = true; d->main_ms = t_ms; }
}

/* The drogue rate: the fast-biased model rate, or the last trustworthy
 * descent rate if faster and measured under a settled drogue. */
static float drogue_rate(const backup_dr *d)
{
  const float measured = -d->ref_vel;
  if (d->drogue && !d->main && reached(d->ref_ms, d->drogue_ms + DR_DROGUE_SETTLE_MS) &&
      finite_f(measured) && measured > DR_DROGUE_MPS)
    return measured < DR_MAX_MPS ? measured : DR_MAX_MPS;
  return DR_DROGUE_MPS;
}

float backup_dr_step(backup_dr *d, uint32_t t_ms, float fallback_alt, float *vel)
{
  if (!d->active)
  {
    if (!d->have_ref) backup_dr_trust(d, t_ms, finite_f(fallback_alt) ? fallback_alt : 0.0f, 0.0f, false);
    d->alt = d->ref_alt;
    d->fall = d->ref_vel < 0.0f ? -d->ref_vel : 0.0f;
    if (d->fall > DR_MAX_MPS) d->fall = DR_MAX_MPS;
    d->rate = drogue_rate(d);
    d->last_ms = d->ref_ms;
    /* Still climbing at most until its earliest apogee. */
    d->climb_until = d->ref_ms + (d->ref_vel > 0.0f
                     ? (uint32_t)(d->ref_vel / DR_DECEL_MAX * 1000.0f) : 0U);
    d->active = true;
  }
  /* From the reference (or the last step) to now, in substeps; a
   * reference older than DR_SPAN_MAX_MS descends from then on. */
  if ((int32_t)(t_ms - d->last_ms) > (int32_t)DR_SPAN_MAX_MS) d->last_ms = t_ms - DR_SPAN_MAX_MS;
  while ((int32_t)(t_ms - d->last_ms) > 0)
  {
    uint32_t step = t_ms - d->last_ms;
    if (step > DR_SUBSTEP_MS) step = DR_SUBSTEP_MS;
    d->last_ms += step;
    if (!reached(d->last_ms, d->climb_until)) continue;
    const float h = (float)step * 0.001f;
    if (d->main && reached(d->last_ms, d->main_ms))
    {
      const float slowed = d->fall - DR_DROGUE_DECEL * h;
      d->fall = slowed > DR_MAIN_MPS ? slowed : DR_MAIN_MPS;
    }
    else if (d->drogue && reached(d->last_ms, d->drogue_ms))
    {
      const float slowed = d->fall - DR_DROGUE_DECEL * h;
      d->fall = slowed > d->rate ? slowed : d->rate;
    }
    else
    {
      const float faster = d->fall + DR_G * h;
      d->fall = faster < DR_MAX_MPS ? faster : DR_MAX_MPS;
    }
    d->alt -= d->fall * h;
    /* On the pad baseline the vehicle has landed. */
    if (d->alt <= 0.0f) { d->alt = 0.0f; d->fall = 0.0f; }
  }
  *vel = -d->fall;
  return d->alt;
}
