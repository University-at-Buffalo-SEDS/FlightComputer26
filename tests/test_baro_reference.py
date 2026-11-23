"""Barometer ground reference: drift tracking on the pad, freeze at Launch.

Runs the actual Core/Src/baro_reference.c over a host fixture fed with the
FC's sample rates (barometer 25 Hz, IMU 100 Hz) and sensor noise, and checks
the live-path wiring in dma.c, barometer.c and board_health.c.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

BASE = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "baro_reference.h"
#include "fc_estimator_status.h"

static uint32_t now;
static uint32_t rng = 12345u;
static float noise(float amp) /* uniform in [-amp, amp] */
{
  rng = rng * 1664525u + 1013904223u;
  return amp * ((float)(rng >> 8) / 8388608.0f - 1.0f);
}
static float alt(float p, float p0) { return baro_agl(p, p0); }

/* Plant */
static double pad;           /* true ambient pressure at the vehicle (Pa) */
static double ramp;          /* Pa per second */
static float extra;          /* transient added to the barometer (Pa) */
static float shake, rate;    /* accelerometer jitter (g), gyro rate (dps) */
static float baro_noise = 1.5f;
static float tilt;           /* rad about x */
static int imu_on = 1;

static float run(uint32_t ms, int on_pad)
{
  float ref = baro_ref.ref;
  for (uint32_t end = now + ms; now < end; ) {
    now += 10u;
    pad += ramp * 0.01;
    if (imu_on) {
      const float a[3] = {noise(0.004f) + noise(shake), sinf(tilt) + noise(0.004f) + noise(shake),
                          cosf(tilt) + noise(0.004f) + noise(shake)};
      const float g[3] = {0.3f + noise(0.2f) + rate, -0.2f + noise(0.2f), noise(0.2f)};
      /* The FC's accelerometer reports m/s^2. */
      baro_ref_accel(a[0] * BARO_REF_G, a[1] * BARO_REF_G, a[2] * BARO_REF_G, now);
      baro_ref_gyro(g[0], g[1], g[2], now);
    }
    if (now % 40u == 0u) ref = baro_ref_update((float)pad + extra + noise(baro_noise), now, on_pad);
  }
  return ref;
}

static void fresh(double p)
{
  memset(&baro_ref, 0, sizeof baro_ref);
  pad = p; ramp = 0; extra = 0; shake = 0; rate = 0; tilt = 0; imu_on = 1;
  baro_ref_reset((float)p, now);
  run(5000u, 1);
}

int main(void)
{
  now = 1000u;

  /* 1. A slow weather ramp of 3 hPa/h for 2 h on a still pad: AGL stays near
   *    0 m; a fixed reference would read about 50 m. */
  fresh(100000.0);
  assert(baro_ref_mode() == BARO_REF_TRACKING);
  ramp = -300.0 / 3600.0;
  float worst = 0.0f;
  for (int minute = 0; minute < 120; ++minute) {
    run(60000u, 1);
    if (minute >= 2) {
      const float a = fabsf(alt((float)pad, baro_ref.ref));
      worst = a > worst ? a : worst;
    }
  }
  const float fixed = alt((float)pad, baro_ref.zero);
  printf("ramp: worst |AGL| %.3f m, fixed reference %.1f m, shift %.1f Pa\n", worst, fixed, baro_ref.ref - baro_ref.zero);
  assert(worst < 0.3f && fixed > 45.0f);
  assert(baro_ref_mode() == BARO_REF_TRACKING);
  /* A noiseless sensor still tracks: each 25 Hz increment is far below a
   *    float step at 1e5 Pa, so the tracker works relative to the zero. */
  fresh(100000.0);
  baro_noise = 0.0f;
  ramp = -300.0 / 3600.0;
  run(1800000u, 1);
  printf("quiet sensor: AGL %.3f m\n", alt((float)pad, baro_ref.ref));
  assert(fabsf(alt((float)pad, baro_ref.ref)) < 0.3f);
  baro_noise = 1.5f;
  /* Rising pressure too (a fast front, 6 hPa/h for 1 h). */
  fresh(95000.0);
  ramp = 600.0 / 3600.0;
  run(3600000u, 1);
  printf("front: AGL %.3f m\n", alt((float)pad, baro_ref.ref));
  assert(fabsf(alt((float)pad, baro_ref.ref)) < 0.6f);

  /* 2. Handling pauses tracking and nothing it causes is absorbed. */
  fresh(100000.0);
  ramp = -300.0 / 3600.0;
  run(60000u, 1);
  float before = baro_ref.ref;
  shake = 0.08f;
  run(20000u, 1);
  assert(baro_ref_mode() == BARO_REF_PAUSED && baro_ref.ref == before);
  shake = 0.0f;
  run(1500u, 1);
  assert(baro_ref_mode() == BARO_REF_PAUSED); /* quiet for 2 s first */
  run(1000u, 1);
  assert(baro_ref_mode() == BARO_REF_TRACKING);
  /*    Tilting (gyro rate, then a held tilt: the 1 g check still passes). */
  before = baro_ref.ref;
  rate = 15.0f;
  for (int i = 0; i < 100; ++i) { tilt += 0.005f; run(20u, 1); }
  assert(baro_ref_mode() == BARO_REF_PAUSED && baro_ref.ref == before);
  rate = 0.0f;
  run(3000u, 1);
  assert(baro_ref_mode() == BARO_REF_TRACKING);
  /*    A tilt without gyro rate (a slow lean) is caught by the accelerometer. */
  before = baro_ref.ref;
  tilt += 0.1f;
  run(100u, 1);
  assert(baro_ref_mode() == BARO_REF_PAUSED && baro_ref.ref == before);
  run(3000u, 1);
  /*    The vehicle laid down or held at an angle that changes |a|. */
  imu_on = 0;
  run(500u, 1);
  assert(baro_ref_mode() == BARO_REF_PAUSED); /* no IMU: not known still */
  imu_on = 1;
  run(3000u, 1);
  assert(baro_ref_mode() == BARO_REF_TRACKING);

  /* 3. Door slam / gust: a 40 Pa step for 1 s moves the reference by at most
   *    the slew limit over the detection delay. */
  fresh(100000.0);
  before = baro_ref.ref;
  extra = 40.0f;
  run(1000u, 1);
  extra = 0.0f;
  run(30000u, 1);
  printf("slam: moved %.3f Pa\n", baro_ref.ref - before);
  assert(fabsf(baro_ref.ref - before) < 0.5f);
  /*    A lasting step (HVAC, a closed door) is absorbed only at the slew
   *    limit, never as a jump. */
  before = baro_ref.ref;
  extra = 40.0f;
  for (int i = 1; i <= 60; ++i) {
    run(1000u, 1);
    assert(baro_ref.ref - before <= BARO_REF_SLEW_PA_S * (float)i + 0.01f);
  }
  extra = 0.0f;

  /* 4. A real slow climb (2 m in 20 s) with a still IMU is not zeroed out. */
  fresh(100000.0);
  ramp = -24.0 / 20.0;
  run(20000u, 1);
  ramp = 0.0;
  run(1000u, 1);
  printf("climb: AGL %.2f m\n", alt((float)pad, baro_ref.ref));
  assert(alt((float)pad, baro_ref.ref) > 1.6f);

  /* 5. Launch freezes the reference at its value from 1.5 s before, so the
   *    motor's pressure effects in the last second are not in it. */
  fresh(100000.0);
  ramp = -300.0 / 3600.0;
  run(120000u, 1);
  /* Ignition: the vehicle is still held, the pressure moves 6 Pa. */
  float history[8]; uint32_t when[8];
  for (int i = 0; i < 8; ++i) { history[i] = baro_ref.ref; when[i] = now; run(250u, 1); }
  extra = -6.0f; ramp = -60.0; /* a fast trend the gate allows for a moment */
  float last = run(1000u, 1);
  (void)last;
  const float at_detect = baro_ref.ref;
  float frozen = run(40u, 0);
  assert(baro_ref_mode() == BARO_REF_FROZEN && baro_ref.frozen == 1.0f);
  /* The reference kept 1.5..2.3 s before detection (sampled every 250 ms). */
  int match = -1; float best = 1e9f;
  for (int i = 0; i < 8; ++i) {
    const uint32_t age = now - 40u - when[i];
    if (age >= 1500u && age <= 2300u && fabsf(history[i] - frozen) < best) { best = fabsf(history[i] - frozen); match = i; }
  }
  printf("freeze: %.3f Pa (at detection %.3f), within %.4f Pa of the reference %u ms before detection\n", frozen,
         at_detect, best, match >= 0 ? (unsigned)(now - 40u - when[match]) : 0u);
  assert(match >= 0 && best < 0.02f);
  assert(fabsf(at_detect - frozen) > 0.05f);

  /* 6. Never updated in flight: climb, transients, a still IMU at apogee. */
  ramp = -1200.0; extra = 0.0f;
  run(10000u, 0);
  ramp = 0.0;
  run(30000u, 0);
  ramp = 80.0;
  run(120000u, 0);
  assert(baro_ref.ref == frozen && baro_ref_mode() == BARO_REF_FROZEN);
  /*    The bits sent in BOARD_HEALTH word 15. */
  uint32_t bits = baro_ref_bits();
  assert((bits & 3u) == BARO_REF_FROZEN);
  int drift = (int)(bits << 18) >> 20; /* sign-extend bits 2..13 */
  assert(drift == (int)lroundf(baro_ref.ref - baro_ref.boot));
  assert(FC_EST_BARO_FROZEN(FC_EST_WORD15(0u, bits)));
  /*    A watchdog resume restores the kept part only: still frozen. */
  baro_ref_kept kept = baro_ref;
  assert(baro_ref_mode() == BARO_REF_FROZEN);
  baro_ref = kept;
  run(1000u, 0);
  assert(baro_ref.ref == frozen);

  /* 7. Back before Launch (a new flight): tracking resumes from there. */
  pad = frozen; ramp = 0.0;
  run(5000u, 1);
  assert(baro_ref_mode() == BARO_REF_TRACKING && baro_ref.frozen == 0.0f);

  /* 8. Freeze right after boot with no history: the current reference. */
  memset(&baro_ref, 0, sizeof baro_ref);
  baro_ref_reset(99000.0f, now);
  run(40u, 0);
  assert(baro_ref.ref == 99000.0f && baro_ref_mode() == BARO_REF_FROZEN);

  /* 9. Shift since boot saturates on the wire; a re-zero keeps the boot one. */
  memset(&baro_ref, 0, sizeof baro_ref);
  baro_ref_reset(100000.0f, now);
  baro_ref_reset(97000.0f, now);
  assert(baro_ref.boot == 100000.0f && baro_ref.zero == 97000.0f);
  bits = baro_ref_bits();
  assert((int)(bits << 18) >> 20 == -2047);
  memset(&baro_ref, 0, sizeof baro_ref);
  assert(baro_ref_bits() == 0u && baro_ref_mode() == BARO_REF_NONE);
  return 0;
}
'''

FLIGHT = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "baro_reference.h"
#include "fcconfig.h"

/* A long pad sit with weather moving the pressure, then a flight, through the
 * FC's live path: baro_reference.c for the reference and baro_agl() for the
 * altitude, at pads from 110 m to 2000 m MSL in an ISA atmosphere; Launch is the evaluator's detect_boost rule (AGL at
 * least LAUNCH_MIN_ALT and rising for MIN_SAMP_LAUNCH samples); the main
 * opens where the barometric AGL first falls to REEF_TARGET_ALT. Compared with
 * a reference fixed at the zero (no compensation). */
static float agl(double p, float p0) { return baro_agl((float)p, p0); }
/* ISA truth: the pressure h m above the level where it is p0. */
static double isa_h(double p) { return 288.15 / 0.0065 * (1.0 - pow(p / 101325.0, 287.05287 * 0.0065 / 9.80665)); }
static double isa_p(double h) { return 101325.0 * pow(1.0 - h * 0.0065 / 288.15, 9.80665 / (287.05287 * 0.0065)); }
static double pressure(double h, double p0) { return isa_p(isa_h(p0) + h); }
static uint32_t rng = 7u;
static float noise(float amp) { rng = rng * 1664525u + 1013904223u; return amp * ((float)(rng >> 8) / 8388608.0f - 1.0f); }

typedef struct { float apogee_err, main_true, launch_t; } result;

static result fly(double hours, double hpa_per_h, int compensate, double pad_msl)
{
  memset(&baro_ref, 0, sizeof baro_ref);
  uint32_t now = 1000u;
  double ground = isa_p(pad_msl);        /* true ground pressure */
  const double rate = hpa_per_h * 100.0 / 3600.0;
  baro_ref_reset((float)ground, now);    /* Postinit's zero */
  const float zero = baro_ref.ref;
  double h = 0.0, v = 0.0, t = 0.0;
  int phase = 0, launched = 0, rising = 0; /* 0 pad, 1 boost, 2 coast, 3 drogue, 4 main, 5 landed */
  float prev = 0.0f, max_agl = -1e9f, h_apogee = 0.0f;
  result r = {0, -1.0f, -1.0f};
  const double sit = hours * 3600.0;
  while (phase < 5) {
    now += 10u; t += 0.01; ground += rate * 0.01;
    double a = 0.0;
    if (phase == 0 && t >= sit) phase = 1;
    if (phase == 1) { a = 9.5 * 9.80665 - 9.80665; if (t >= sit + 3.6) phase = 2; }
    if (phase == 2) { a = -9.80665 - 0.0007 * v * fabs(v); if (v <= 0.0) { phase = 3; h_apogee = (float)h; } }
    if (phase == 3 || phase == 4) { const double vt = phase == 3 ? -25.0 : -6.0; a = (vt - v) * 2.0; }
    if (phase) { v += a * 0.01; h += v * 0.01; if (h <= 0.0 && phase >= 3) phase = 5; }
    const float f = phase ? (float)(a / 9.80665) + 1.0f : 1.0f;
    baro_ref_accel(noise(0.003f) * BARO_REF_G, noise(0.003f) * BARO_REF_G, (f + noise(0.003f)) * BARO_REF_G, now);
    baro_ref_gyro(noise(0.3f), noise(0.3f), noise(0.3f), now);
    if (now % 40u) continue;
    /* The igniter and motor start disturb the pressure in the last second on the pad. */
    const double p = pressure(h, ground) + (phase == 0 && t > sit - 1.0 ? -12.0 : 0.0) + noise(1.5f);
    float ref = compensate ? baro_ref_update((float)p, now, !launched) : zero;
    const float alt = agl(p, ref);
    rising = alt >= LAUNCH_MIN_ALT && alt > prev ? rising + 1 : 0;
    if (!launched && rising >= MIN_SAMP_LAUNCH) { launched = 1; r.launch_t = (float)(t - sit); }
    if (phase == 0 && t > sit - 0.05) r.apogee_err = alt; /* reused below: AGL on the pad at ignition */
    prev = alt;
    if (alt > max_agl) max_agl = alt;
    if (phase == 3 && alt <= REEF_TARGET_ALT) { phase = 4; r.main_true = (float)h; }
    if (phase == 5) break;
  }
  printf("pad %4.0f m MSL, %s %.1f h at %+.1f hPa/h: pad AGL at ignition %+.2f m, launch detected T+%.2f s, "
         "apogee %.1f m read as %.1f m, main at true %.1f m (target %.1f m)\n",
         pad_msl, compensate ? "compensated" : "fixed ref  ", hours, hpa_per_h, r.apogee_err, r.launch_t,
         h_apogee, max_agl, r.main_true, REEF_TARGET_ALT);
  r.apogee_err = max_agl - h_apogee;
  return r;
}

int main(void)
{
  /* hours, hPa/h, pad m MSL (110.9 m: 100000 Pa) */
  const double cases[][3] = {{2.0, -3.0, 110.9}, {1.0, +4.0, 110.9}, {4.0, -1.5, 110.9},
                             {2.0, -3.0, 890.0}, {1.0, +4.0, 1500.0}, {4.0, -1.5, 2000.0}};
  for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
    result good = fly(cases[i][0], cases[i][1], 1, cases[i][2]);
    result bad = fly(cases[i][0], cases[i][1], 0, cases[i][2]);
    assert(fabsf(good.apogee_err) < 3.0f);
    assert(fabsf(good.main_true - REEF_TARGET_ALT) < 3.0f);
    assert(good.launch_t > 0.0f && good.launch_t < 1.0f);
    /* Without compensation the error is the weather's 8 m (or more, higher up) per hPa. */
    const float drift_m = (float)(fabs(cases[i][0] * cases[i][1]) * 8.0);
    assert(fabsf(bad.apogee_err) > 0.7f * drift_m || fabsf(bad.main_true - REEF_TARGET_ALT) > 0.7f * drift_m);
  }
  return 0;
}
"""

ALTITUDE = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "baro_reference.h"
#include "fcconfig.h"

/* baro_agl() against ISA truth (double, from the constants' definitions) at
 * pads from sea level to 2000 m MSL, 0-5000 m above them. */
static double isa_h(double p) { return 288.15 / 0.0065 * (1.0 - pow(p / 101325.0, 287.05287 * 0.0065 / 9.80665)); }
static double isa_p(double h) { return 101325.0 * pow(1.0 - h * 0.0065 / 288.15, 9.80665 / (287.05287 * 0.0065)); }
static float sea_level_form(float p, float p0) { return 44330.0f * (1.0f - powf(p / p0, 0.1903f)); }

int main(void)
{
  const double pads[] = {0.0, 890.016, 1500.0, 2000.0};
  for (unsigned i = 0; i < sizeof pads / sizeof pads[0]; ++i) {
    const float p0 = (float)isa_p(pads[i]);
    double worst = 0.0, worst_rel = 0.0, old_rel = 0.0;
    for (double h = 0.0; h <= 5000.0; h += 5.0) {
      const float p = (float)isa_p(pads[i] + h);
      const double got = baro_agl(p, p0);
      /* The truth for the float pressures handed over. */
      const double truth = isa_h(p) - isa_h(p0);
      const double err = got - truth;
      if (fabs(err) > worst) worst = fabs(err);
      if (h >= 100.0) {
        if (fabs(err) / h > worst_rel) worst_rel = fabs(err) / h;
        const double old = (sea_level_form(p, p0) - truth) / h;
        if (fabs(old) > fabs(old_rel)) old_rel = old;
      }
      assert(fabs(err) < 0.05 + 2e-5 * h);
      /* Also against the nominal height (pressure rounded to float). */
      assert(fabs(got - h) < 0.05 + 2e-5 * h);
    }
    /* The main: the first ISA pressure at or below REEF_TARGET_ALT, coming
     * down in 1 cm steps, is the right true height. */
    double cross = -1.0;
    for (double h = 600.0; h >= 0.0; h -= 0.01) {
      if (baro_agl((float)isa_p(pads[i] + h), p0) <= REEF_TARGET_ALT) { cross = h; break; }
    }
    printf("pad %7.1f m MSL (%.1f Pa): worst |err| %.3f m (%.4f%%), main at true %.2f m; "
           "sea-level form off by %+.2f%%\n",
           pads[i], (double)p0, worst, worst_rel * 100.0, cross, old_rel * 100.0);
    assert(fabs(cross - REEF_TARGET_ALT) < 0.05);
    if (pads[i] > 800.0) assert(old_rel > 0.018); /* the bug this replaces */
  }
  /* The cached pad factor follows a changing reference. */
  const float a = (float)isa_p(0.0), b = (float)isa_p(2000.0), p = (float)isa_p(3000.0);
  const float ab = baro_agl(p, b), aa = baro_agl(p, a), ab2 = baro_agl(p, b);
  assert(ab == ab2 && fabsf(ab - 1000.0f) < 0.1f && fabsf(aa - 3000.0f) < 0.1f);
  return 0;
}
"""


class BaroReferenceTests(unittest.TestCase):
    def test_tracking_gates_and_freeze(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out / 'h.c').write_text(HARNESS)
            exe = out / 'probe'
            subprocess.run(['cc', '-std=gnu11', '-O1', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-I', str(BASE / 'Core/Inc'),
                            str(out / 'h.c'), str(BASE / 'Core/Src/baro_reference.c'),
                            '-o', str(exe), '-lm'], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=120)
            print(result.stdout)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_long_pad_sit_then_flight_against_a_fixed_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out / 'f.c').write_text(FLIGHT)
            exe = out / 'flight'
            subprocess.run(['cc', '-std=gnu11', '-O1', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-I', str(BASE / 'Core/Inc'),
                            str(out / 'f.c'), str(BASE / 'Core/Src/baro_reference.c'),
                            '-o', str(exe), '-lm'], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=300)
            print(result.stdout)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_altitude_at_any_pad_elevation(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            (out / 'a.c').write_text(ALTITUDE)
            exe = out / 'alt'
            subprocess.run(['cc', '-std=gnu11', '-O1', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', '-I', str(BASE / 'Core/Inc'),
                            str(out / 'a.c'), str(BASE / 'Core/Src/baro_reference.c'),
                            '-o', str(exe), '-lm'], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=60)
            print(result.stdout)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_live_path_wiring(self):
        dma = (BASE / 'Core/Src/dma.c').read_text()
        # Every fresh live sample: the reference sees the real sensors, and
        # Launch (or any later state) freezes it.
        self.assertIn('buf->alt = baro_track_alt(buf->prs, current() < Launch);', dma)
        self.assertIn('baro_ref_accel(buf->x, buf->y, buf->z, now_ms());', dma)
        self.assertIn('baro_ref_gyro(buf->x, buf->y, buf->z, now_ms());', dma)
        # Units the stationarity limits assume: m/s^2 (despite the name) and deg/s.
        accel = (BASE / 'Core/Src/accelerometer.c').read_text()
        self.assertIn('lsb_to_g = accl_lsb_to_mps2(range);', accel)
        self.assertIn('1.0f/16.384f', (BASE / 'Core/Src/gyroscope.c').read_text())
        baro = (BASE / 'Core/Src/barometer.c').read_text()
        # The kept reference first: the resume check reads it as a pressure.
        regions = baro[baro.index('fc_resume_barometer[] = {'):]
        self.assertLess(regions.index('FC_RESUME_REGION(baro_ref)'), regions.index('FC_RESUME_REGION(prev_rel_alt)'))
        self.assertIn('baro_reset_baseline(p);', baro)
        self.assertNotIn('alt_bias', baro)
        # One pressure-to-altitude conversion, the site-elevation-correct one.
        self.assertIn('float rel_alt = baro_agl(pressure, baro_ref.ref);', baro)
        self.assertNotIn('44330', baro)
        self.assertNotIn('44330', (BASE / 'Core/Inc/barometer.h').read_text())
        cmake = (BASE / 'CMakeLists.txt').read_text()
        self.assertIn('Core/Src/baro_reference.c', cmake)


if __name__ == '__main__':
    unittest.main()
