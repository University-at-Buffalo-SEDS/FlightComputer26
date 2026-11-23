/* Barometer ground reference tracking and freeze (baro_reference.h). */

#include "baro_reference.h"
#include <math.h>
#include <string.h>

baro_ref_kept baro_ref;

/* Runtime only: a resume restarts the averages and the history. Pressures
 * here are relative to the last zero: at 1e5 Pa a float step is 0.008 Pa,
 * coarser than one 25 Hz tracking increment. */
static struct {
  float a[3];                 /* accelerometer average (m/s^2) */
  float off;                  /* reference - zero (Pa) */
  float fast, slow;           /* pressure averages - zero (Pa) */
  float ring[BARO_REF_RING];  /* past off */
  uint32_t accel_ms, gyro_ms, press_ms, ring_ms;
  uint32_t calm_since;        /* last disturbance */
  uint8_t accel_ok, press_ok, ring_n, ring_i, mode;
} s;

static inline float ema_gain(uint32_t dt, uint32_t tau)
{
  return (float)dt / (float)(tau + dt);
}

static inline void disturbed(uint32_t now) { s.calm_since = now; }

float baro_agl(float p, float p0)
{
  static float key, scale; /* p0 and T_pad/L for it */
  if (p0 != key) {
    scale = BARO_ISA_T0_L * powf(p0 * (1.0f / BARO_ISA_P0), BARO_ISA_EXP);
    key = p0;
  }
  return scale * (1.0f - powf(p / p0, BARO_ISA_EXP));
}

void baro_ref_reset(float p, uint32_t now)
{
  if (!(baro_ref.boot > 0.0f)) baro_ref.boot = p;
  baro_ref.ref = baro_ref.zero = p;
  baro_ref.frozen = 0.0f;
  s.press_ok = s.ring_n = 0;
  s.mode = BARO_REF_PAUSED;
  disturbed(now);
}

void baro_ref_accel(float x, float y, float z, uint32_t now)
{
  const float a[3] = {x, y, z};
  const uint32_t dt = now - s.accel_ms;
  s.accel_ms = now;
  const float m2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
  const float lo = (1.0f - BARO_REF_ACC_MAG_G) * BARO_REF_G;
  const float hi = (1.0f + BARO_REF_ACC_MAG_G) * BARO_REF_G;
  bool calm = m2 >= lo * lo && m2 <= hi * hi;

  if (!s.accel_ok || dt > BARO_REF_IMU_STALE_MS) {
    memcpy(s.a, a, sizeof s.a);
    s.accel_ok = 1;
    calm = false;
  }
  const float k = ema_gain(dt, BARO_REF_ACC_AVG_MS);
  for (unsigned i = 0; i < 3; ++i) {
    const float d = a[i] - s.a[i];
    if (fabsf(d) > BARO_REF_ACC_JITTER_G * BARO_REF_G) calm = false;
    s.a[i] += k * d;
  }
  if (!calm) disturbed(now);
}

void baro_ref_gyro(float x, float y, float z, uint32_t now)
{
  if (now - s.gyro_ms > BARO_REF_IMU_STALE_MS ||
      fabsf(x) > BARO_REF_GYRO_DPS || fabsf(y) > BARO_REF_GYRO_DPS ||
      fabsf(z) > BARO_REF_GYRO_DPS)
  {
    disturbed(now);
  }
  s.gyro_ms = now;
}

static void freeze(void)
{
  /* The entry k back was kept at least (k - 1) ring periods ago. */
  unsigned k = BARO_REF_LOOKBACK_MS / BARO_REF_RING_MS + 1u;
  if (k > s.ring_n) k = s.ring_n;
  if (k) {
    s.off = s.ring[(s.ring_i + BARO_REF_RING - k) % BARO_REF_RING];
    baro_ref.ref = baro_ref.zero + s.off;
  }
  baro_ref.frozen = 1.0f;
}

float baro_ref_update(float p, uint32_t now, bool on_pad)
{
  if (!(baro_ref.ref > 0.0f)) baro_ref_reset(p, now);

  if (!on_pad) {
    if (baro_ref.frozen == 0.0f) freeze();
    s.mode = BARO_REF_FROZEN;
    return baro_ref.ref;
  }
  if (baro_ref.frozen != 0.0f) {
    /* Back before Launch: a new flight. */
    baro_ref.frozen = 0.0f;
    s.press_ok = s.ring_n = 0;
  }

  const float q = p - baro_ref.zero;
  const uint32_t dt = now - s.press_ms;
  s.press_ms = now;
  if (!s.press_ok || dt > BARO_REF_GAP_MS) {
    s.off = baro_ref.ref - baro_ref.zero;
    s.fast = s.slow = q;
    s.press_ok = 1;
    s.ring_n = 0;
    s.ring_ms = now - BARO_REF_RING_MS;
    disturbed(now);
  } else {
    s.fast += ema_gain(dt, BARO_REF_FAST_MS) * (q - s.fast);
    s.slow += ema_gain(dt, BARO_REF_SLOW_MS) * (q - s.slow);
    if (fabsf(s.fast - s.slow) > BARO_REF_STEP_PA) disturbed(now);
  }

  const bool still = s.accel_ok &&
                     now - s.accel_ms <= BARO_REF_IMU_STALE_MS &&
                     now - s.gyro_ms <= BARO_REF_IMU_STALE_MS &&
                     now - s.calm_since >= BARO_REF_QUIET_MS;
  if (still) {
    const float limit = BARO_REF_SLEW_PA_S * 0.001f * (float)dt;
    float step = ema_gain(dt, BARO_REF_TAU_MS) * (s.fast - s.off);
    step = step > limit ? limit : step < -limit ? -limit : step;
    const float off = s.off + step;
    s.off = off < -BARO_REF_MAX_SHIFT_PA ? -BARO_REF_MAX_SHIFT_PA
          : off > BARO_REF_MAX_SHIFT_PA ? BARO_REF_MAX_SHIFT_PA : off;
    baro_ref.ref = baro_ref.zero + s.off;
  }
  s.mode = still ? BARO_REF_TRACKING : BARO_REF_PAUSED;

  if (now - s.ring_ms >= BARO_REF_RING_MS) {
    s.ring_ms = now;
    s.ring[s.ring_i] = s.off;
    s.ring_i = (uint8_t)((s.ring_i + 1u) % BARO_REF_RING);
    if (s.ring_n < BARO_REF_RING) ++s.ring_n;
  }
  return baro_ref.ref;
}

uint8_t baro_ref_mode(void)
{
  if (!(baro_ref.ref > 0.0f)) return BARO_REF_NONE;
  /* After a resume the frozen flag is restored before any sample. */
  return baro_ref.frozen != 0.0f ? BARO_REF_FROZEN
       : s.mode == BARO_REF_TRACKING ? BARO_REF_TRACKING : BARO_REF_PAUSED;
}

uint32_t baro_ref_bits(void)
{
  if (!(baro_ref.ref > 0.0f)) return 0u;
  float d = baro_ref.ref - baro_ref.boot;
  d = d > 2047.0f ? 2047.0f : d < -2047.0f ? -2047.0f : d;
  const int32_t drift = (int32_t)(d < 0.0f ? d - 0.5f : d + 0.5f);
  return (((uint32_t)drift & 0xFFFu) << 2) | baro_ref_mode();
}
