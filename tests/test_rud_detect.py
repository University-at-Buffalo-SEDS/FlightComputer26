"""Break-up (RUD) abort detector, rud_detect.c, on the host.

Pins the trigger rule: the IMU (tumble or lateral load) on at least
RUD_MOTION_MIN and the barometer (implausible altitude rate) on at least
RUD_PRESSURE_MIN of the last RUD_WINDOW steps, both at once, while armed;
never one sensor alone, never a transient, fires once, and disarming forgets
every run. The sweep over flight data is tools/rud_sweep.py."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <math.h>
#include "rud_detect.h"
static rud_state s; static unsigned t; static float alt;
/* One 40 ms step; motion: tumbling IMU; jump: barometer moves 30 m (750 m/s). */
static bool step(bool armed, bool motion, bool jump, float lateral)
{
  t += 40; alt += jump ? ((t / 40) % 2 ? 30.0f : -30.0f) : 1.0f;
  rud_input in = { .t_ms = t, .armed = armed, .gyro = { motion ? 400.0f : 10.0f, 0, 50 },
                   .accel = { lateral, 0, 9.8f }, .baro_alt = alt };
  return rud_step(&s, &in);
}
static unsigned run(unsigned n, bool motion, bool jump) { unsigned fired = 0; for (unsigned i = 0; i < n; ++i) fired += step(true, motion, jump, 0); return fired; }
int main(void)
{
  /* Thresholds as documented. */
  assert(RUD_TUMBLE_DPS == 360.0f && RUD_LATERAL_MPS2 == 49.0f && RUD_PRESSURE_RATE_MPS == 450.0f);
  assert(RUD_WINDOW == 25U && RUD_MOTION_MIN == 22U && RUD_PRESSURE_MIN == 12U);
  /* One sensor alone never fires, however long. */
  rud_reset(&s); assert(run(200, true, false) == 0);
  rud_reset(&s); assert(run(200, false, true) == 0);
  /* Both for less than the window: no; held: once, on the step it holds. */
  rud_reset(&s); assert(run(RUD_MOTION_MIN - 1, true, true) == 0);
  rud_reset(&s); { unsigned i = 0; while (!step(true, true, true, 0)) ++i; assert(i + 1 == RUD_WINDOW); }
  assert(run(100, true, true) == 0); /* fires once */
  /* Lateral load counts as IMU motion. */
  rud_reset(&s); { unsigned n = 0; for (unsigned i = 0; i < RUD_WINDOW; ++i) n += step(true, false, true, 60.0f); assert(n == 1); }
  /* Transients interleaved below the vote counts never fire. */
  rud_reset(&s); { unsigned n = 0; for (unsigned i = 0; i < 400; ++i) n += step(true, i % 25 < 21, i % 25 < 24, 0); assert(n == 0); }
  rud_reset(&s); { unsigned n = 0; for (unsigned i = 0; i < 400; ++i) n += step(true, true, i % 25 < 11, 0); assert(n == 0); }
  /* Disarmed: nothing counts, and arming starts from scratch. */
  rud_reset(&s); for (unsigned i = 0; i < 100; ++i) assert(!step(false, true, true, 0));
  for (unsigned i = 0; i < RUD_WINDOW - 1; ++i) assert(!step(true, true, true, 0));
  assert(!step(false, true, true, 0));
  for (unsigned i = 0; i < RUD_WINDOW - 1; ++i) assert(!step(true, true, true, 0));
  assert(step(true, true, true, 0));
  /* Fired stays fired across disarming. */
  assert(!step(false, true, true, 0)); assert(run(100, true, true) == 0);
  /* Non-finite readings never vote. */
  rud_reset(&s); { unsigned n = 0; for (unsigned i = 0; i < 100; ++i) {
      t += 40; rud_input in = { .t_ms = t, .armed = true, .gyro = { NAN, NAN, 0 }, .accel = { INFINITY, 0, 0 }, .baro_alt = NAN };
      n += rud_step(&s, &in); } assert(n == 0); }
  return 0;
}
'''


class RudDetect(unittest.TestCase):
    def test_trigger_rule(self):
        with tempfile.TemporaryDirectory() as tmp:
            src, exe = Path(tmp) / 'r.c', Path(tmp) / 'r'
            src.write_text(HARNESS)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            f'-I{ROOT / "Core/Inc"}', str(src), str(ROOT / 'Core/Src/rud_detect.c'), '-o', str(exe), '-lm'],
                           check=True)
            subprocess.run([str(exe)], check=True)

    def test_wired_into_the_estimator(self):
        evaluation = (ROOT / 'Core/Src/estimator_thread.c').read_text()
        watch = evaluation[evaluation.index('static inline void watch_for_breakup('):]
        watch = watch[:watch.index('\n}\n')]
        self.assertIn('.armed = curr >= Launch && curr <= Coast && (conf & option(Using_Ascent_KF))', watch)
        self.assertIn('fc_recovery_request_eval_stop();', watch)
        body = evaluation[evaluation.index('void evaluate_rocket_state('):]
        self.assertLess(body.index('watch_for_breakup(conf, curr);'), body.index('switch (curr)'))
        self.assertIn('Core/Src/rud_detect.c', (ROOT / 'CMakeLists.txt').read_text())


if __name__ == '__main__':
    unittest.main()
