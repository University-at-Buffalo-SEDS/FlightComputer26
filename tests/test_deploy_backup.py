"""The main without a trustworthy altitude, deploy_backup.c, on the host.

The dead-reckoned descent that times the main when no altitude can be
trusted: it must stay below the true altitude, so the main is early, never
late. Checked here against simulated flights with drag (the
open-loop IGT replay cannot show the free fall before a late drogue). The
sweep over flight data is tools/rud_sweep.py; docs/rud-abort.md section 6."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "deploy_backup.h"

static backup_dr d;

/* A vehicle in coast from (h, v) at t0 with drag k v^2 (0.54 g at 220 m/s,
 * the IGT airframe), free fall with k_fall after apogee, a drogue at
 * t_drogue to a terminal rate v_drogue (drag g (v / v_drogue)^2), the main
 * at the FC's command to 6.5 m/s. Returns the minimum true altitude at
 * which the main fired, or a negative number if it was late (under 457 m,
 * less one 40 ms step at the fast rate and 1 m of rounding when the true
 * drogue rate is the bound itself) or the prediction ever rose above the
 * true altitude. */
static float fly(double h, double v, double t_fail, double t_drogue, double v_drogue, double k_fall, bool clean_ref)
{
  backup_dr_reset(&d);
  const double k = 5.3 / (220.0 * 220.0);
  double t = 0.0;
  const double dt = 0.001;
  bool drogue = false, mainout = false;
  float main_h = -1.0f;
  for (unsigned i = 0; i < 400000; ++i, t += dt) {
    const uint32_t ms = (uint32_t)lround(t * 1000.0);
    /* the truth */
    double a;
    if (v > 0.0f) a = -DR_G - k * v * v;
    else if (mainout) a = -DR_G + DR_G * (v / 6.5f) * (v / 6.5f);
    else if (drogue) a = -DR_G + DR_G * (v / v_drogue) * (v / v_drogue);
    else a = -DR_G + k_fall * v * v;
    v += a * dt; h += v * dt;
    if (h <= 0.0f) break;
    if (!drogue && t >= t_drogue) drogue = true;
    /* the FC: trustworthy until t_fail, then a step every 40 ms */
    if (ms % 40U) continue;
    if (t < t_fail) {
      /* the sensors still time the main */
      if (drogue && !mainout && h <= 457.2) { mainout = true; main_h = (float)h; }
      backup_dr_trust(&d, ms, (float)h, (float)v, clean_ref);
      continue;
    }
    backup_dr_chutes(&d, ms, drogue, mainout);
    float pv;
    const float ph = backup_dr_step(&d, ms, 0.0f, &pv);
    /* 1 m: a true drogue rate on the fast bound itself descends as fast */
    if (ph > h + 1.0f) { printf("prediction %.1f above truth %.1f at %.2f s\n", ph, h, t); return -2.0f; }
    if (drogue && !mainout && ph <= 457.2f) { mainout = true; main_h = (float)h; }
  }
  return main_h >= 457.2f - 2.5f ? main_h : -1.0f;
}

int main(void)
{
  /* Under the drogue at 25 m/s: the prediction descends at 28.75 m/s; the
   * main comes (3000 - 457) / 28.75 s after the reference. */
  backup_dr_reset(&d);
  backup_dr_chutes(&d, 1000U, true, false);
  backup_dr_trust(&d, 10000U, 3000.0f, -25.0f, true);
  float v, h = backup_dr_step(&d, 11000U, 0.0f, &v);
  assert(fabsf(h - (3000.0f - DR_DROGUE_MPS)) < 0.01f && v == -DR_DROGUE_MPS);
  const uint32_t main_ms = 10000U + (uint32_t)((3000.0f - 457.2f) / DR_DROGUE_MPS * 1000.0f);
  h = backup_dr_step(&d, main_ms - 80U, 0.0f, &v); assert(h > 457.2f);
  h = backup_dr_step(&d, main_ms + 80U, 0.0f, &v); assert(h <= 457.2f);
  /* A faster settled descent (a partly open drogue) is kept, up to the cap. */
  backup_dr_trust(&d, 20000U, 2000.0f, -60.0f, true);
  h = backup_dr_step(&d, 21000U, 0.0f, &v); assert(fabsf(h - 1940.0f) < 0.01f);
  /* Under the main, and stopping at the pad. */
  backup_dr_chutes(&d, 22000U, true, true);
  backup_dr_trust(&d, 30000U, 100.0f, -6.5f, true);
  h = backup_dr_step(&d, 31000U, 0.0f, &v); assert(fabsf(h - (100.0f - DR_MAIN_MPS)) < 0.01f);
  h = backup_dr_step(&d, 60000U, 0.0f, &v); assert(h == 0.0f && v == 0.0f);

  /* A reference that is not measured is capped by the no-drag ballistic
   * flight from the last measured one. */
  backup_dr_reset(&d);
  backup_dr_trust(&d, 22000U, 1978.0f, 150.0f, true);
  backup_dr_trust(&d, 40000U, 5000.0f, 300.0f, false);
  assert(d.ref_alt <= 1978.0f + 150.0f * 150.0f / (2.0f * DR_G) + 0.01f && d.ref_vel == 0.0f);
  /* No reference at all (a restart): the fallback altitude. */
  backup_dr_reset(&d);
  h = backup_dr_step(&d, 5000U, 800.0f, &v); assert(h == 800.0f);

  /* Against simulated flights: references in coast (the barometer lost at
   * 22 s on the 10k and the measured-motor flights), at apogee and under
   * the drogue, drogues on time and up to 10 s late, drogue rates 15 % either side of the model, a falling body
   * with little or much drag. The main is never under 457 m. */
  const float coast[][2] = { {1978.0f, 150.0f}, {1240.0f, 72.0f}, {2381.0f, 115.0f} };
  const float rates[] = { 25.0f / 1.15f, 25.0f, 25.0f * 1.15f };
  const float kfall[] = { 0.0005f, 0.004f };
  unsigned flights = 0; float lowest = 1e9f, highest = 0.0f;
  for (unsigned c = 0; c < 3; ++c)
    for (unsigned r = 0; r < 3; ++r)
      for (unsigned f = 0; f < 2; ++f)
        for (float late = 0.0f; late <= 10.0f; late += 2.5f) {
          /* apogee of this coast, from a quick integration */
          float vv = coast[c][1], ta = 0.0f;
          const float k = 5.3f / (220.0f * 220.0f);
          while (vv > 0.0f) { vv += (-DR_G - k * vv * vv) * 0.001f; ta += 0.001f; }
          /* the sensors fail at once (t 0.04 s): an ascent reference */
          const float m = fly(coast[c][0], coast[c][1], 0.04f, ta + 1.2f + late, rates[r], kfall[f], true);
          if (m < 0.0f) { printf("coast %u rate %u fall %u late %.1f: main late\n", c, r, f, late); return 1; }
          lowest = fminf(lowest, m); highest = fmaxf(highest, m); ++flights;
          /* the sensors fail under the drogue, 10 s after it */
          const float m2 = fly(coast[c][0], coast[c][1], ta + 11.2f + late, ta + 1.2f + late, rates[r], kfall[f], true);
          if (m2 < 0.0f) { printf("drogue %u rate %u fall %u late %.1f: main late\n", c, r, f, late); return 1; }
          lowest = fminf(lowest, m2); ++flights;
        }
  printf("%u simulated flights: the main at a true %.0f m or higher (highest %.0f m)\n", flights, lowest, highest);
  return 0;
}
'''


class DeployBackup(unittest.TestCase):
    def test_dead_reckoning(self):
        with tempfile.TemporaryDirectory() as tmp:
            src, exe = Path(tmp) / 'b.c', Path(tmp) / 'b'
            src.write_text(HARNESS)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            f'-I{ROOT / "Core/Inc"}', str(src), str(ROOT / 'Core/Src/deploy_backup.c'),
                            '-o', str(exe), '-lm'], check=True)
            out = subprocess.run([str(exe)], capture_output=True, text=True, timeout=120)
            self.assertEqual(out.returncode, 0, out.stdout + out.stderr)

    def test_wired_into_the_estimator(self):
        est = (ROOT / 'Core/Src/estimator_thread.c').read_text()
        self.assertIn('Core/Src/deploy_backup.c', (ROOT / 'CMakeLists.txt').read_text())
        self.assertIn('Core/Src/deploy_backup.c', (ROOT / 'tools/igt_replay_host.py').read_text())
        descent = est[est.index('void estimator_descent_step('):]
        descent = descent[:descent.index('\n}\n')]
        self.assertIn('descent_dead_reckon(t);', descent)


if __name__ == '__main__':
    unittest.main()
