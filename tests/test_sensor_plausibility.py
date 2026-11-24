"""Single-sensor fallbacks, sensor_plausibility.c, on the host.

One faulty sensor must not cost the chutes: an implausible or frozen
barometer is not used (the filters dead-reckon, or take GPS altitude in
descent), and an ascent velocity that disagrees with its own altitude trend
for a second hands Apogee detection to the barometer. The sweep over flight
data with injected faults is tools/rud_sweep.py."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <math.h>
#include "sensor_plausibility.h"
#include "rud_detect.h"
static plaus_state p; static unsigned t; static float alt;
static bool baro(float step) { t += 40; alt += step; return plaus_baro(&p, t, alt); }
int main(void)
{
  assert(PLAUS_BARO_CLEAR_MAX == 2U && PLAUS_VEL_TOLER_MPS == 40.0f && PLAUS_VEL_STEPS == 25U);
  assert(PLAUS_BARO_GATE_M == 150.0f && PLAUS_BARO_STUCK_STEPS == 50U);
  /* A clean descent at 25 m/s: every sample used, never suspect. */
  plaus_reset(&p); for (unsigned i = 0; i < 300; ++i) assert(baro(-1.0f));
  assert(plaus_baro_ok(&p) && !plaus_baro_suspect(&p));
  /* An outlier: the jump and the return are not used, the rest are; the
   * barometer as a whole stays plausible. */
  assert(!baro(40.0f) && !baro(-40.0f) && plaus_baro_suspect(&p) && plaus_baro_ok(&p));
  assert(baro(-1.0f));
  /* Chaos: implausible after RUD_PRESSURE_MIN fast steps in the window, and
   * plausible again only once the window is nearly clean. */
  plaus_reset(&p); baro(0);
  for (unsigned i = 0; i < RUD_PRESSURE_MIN - 1U; ++i) { assert(!baro(i % 2 ? 300.0f : -300.0f)); assert(plaus_baro_ok(&p)); }
  assert(!baro(300.0f) && !plaus_baro_ok(&p));
  /* The chaos votes leave the 25-step window one clean step at a time; the
   * step that brings the count down to PLAUS_BARO_CLEAR_MAX is used again. */
  unsigned clean = 0; bool used = false;
  while (!used) { used = baro(-1.0f); ++clean; assert(used == plaus_baro_ok(&p)); }
  assert(clean == RUD_WINDOW - PLAUS_BARO_CLEAR_MAX && baro(-1.0f));
  /* Frozen: 2 s of identical altitudes, then implausible until it moves. */
  plaus_reset(&p); baro(0);
  for (unsigned i = 0; i < PLAUS_BARO_STUCK_STEPS - 1U; ++i) assert(baro(0.0f) || i < 1000);
  assert(plaus_baro_ok(&p)); assert(!baro(0.0f) && !plaus_baro_ok(&p));
  assert(baro(-1.0f) && plaus_baro_ok(&p));
  /* Non-finite altitude: a vote, never used. */
  plaus_reset(&p); baro(0); { t += 40; assert(!plaus_baro(&p, t, NAN)); }
  /* Velocity: 24 disagreeing steps keep trust; 25 drop it; 25 agreeing
   * steps restore it. */
  plaus_reset(&p);
  for (unsigned i = 0; i < PLAUS_VEL_STEPS - 1U; ++i) { plaus_velocity(&p, 140.0f, 90.0f); assert(plaus_velocity_ok(&p)); }
  plaus_velocity(&p, 140.0f, 140.0f);                 /* the run breaks */
  for (unsigned i = 0; i < PLAUS_VEL_STEPS - 1U; ++i) plaus_velocity(&p, 140.0f, 90.0f);
  assert(plaus_velocity_ok(&p)); plaus_velocity(&p, 140.0f, 90.0f); assert(!plaus_velocity_ok(&p));
  for (unsigned i = 0; i < PLAUS_VEL_STEPS - 1U; ++i) { plaus_velocity(&p, 100.0f, 95.0f); assert(!plaus_velocity_ok(&p)); }
  plaus_velocity(&p, 100.0f, 95.0f); assert(plaus_velocity_ok(&p));
  plaus_velocity(&p, NAN, 0.0f); /* counts as disagreement, no crash */
  return 0;
}
'''


class SensorPlausibility(unittest.TestCase):
    def test_verdicts(self):
        with tempfile.TemporaryDirectory() as tmp:
            src, exe = Path(tmp) / 'p.c', Path(tmp) / 'p'
            src.write_text(HARNESS)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            f'-I{ROOT / "Core/Inc"}', str(src), str(ROOT / 'Core/Src/sensor_plausibility.c'),
                            '-o', str(exe), '-lm'], check=True)
            subprocess.run([str(exe)], check=True)

    def test_wired_into_the_estimator(self):
        est = (ROOT / 'Core/Src/estimator_thread.c').read_text()
        asc = est[est.index('void estimator_ascent_update(void)'):]
        asc = asc[:asc.index('\n}\n')]
        self.assertIn('plaus_baro_suspect(&plaus)', asc)
        self.assertIn('if (usable) ascent_update();\n  else ascent_hold();', asc)
        des = est[est.index('void estimator_descent_step('):]
        des = des[:des.index('\n}\n')]
        self.assertLess(des.index('descent_update();'), des.index('descent_update_z(z);'))
        self.assertLess(des.index('descent_update_z(z);'), des.index('descent_dead_reckon(t);'))
        self.assertIn('meas.gps.sea - LAUNCH_SITE_SEA', des)
        self.assertIn("((conf & option(Vigilant_Mode)) || !ekf_velocity_trusted())", est)
        self.assertIn('Core/Src/sensor_plausibility.c', (ROOT / 'CMakeLists.txt').read_text())
        # The estimator loop and the IGT host replay take the same steps.
        loop = est[est.index('void estimator_thread_entry('):]
        self.assertIn('estimator_ascent_update();', loop)
        self.assertIn('estimator_descent_step(conf, dt);', loop)
        replay = (ROOT / 'tools/igt_replay_host.py').read_text()
        self.assertIn('estimator_ascent_update();', replay)
        self.assertIn('estimator_descent_step(conf, dt);', replay)


if __name__ == '__main__':
    unittest.main()
