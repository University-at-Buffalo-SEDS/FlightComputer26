"""The telemetry stack low-water scan is rate limited to once per second."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def compile_and_run(test, source, *flags):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)
        (path / 'test.c').write_text(source)
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra',
                        '-Werror', '-Wno-unused-function', *flags,
                        str(path / 'test.c'), '-o', str(path / 'test')], check=True)
        subprocess.run([str(path / 'test')], check=True)


class StackScanRate(unittest.TestCase):
    def test_stack_scan_runs_at_most_once_per_second(self):
        source = (ROOT / 'Core/Src/telemetry.c').read_text()
        start = source.index('#define TELEMETRY_STACK_SCAN_PERIOD_TICKS')
        end = source.index('/* Stack-fill scanning only sees bytes', start)
        production = source[start:end]
        harness = r'''
#include <assert.h>
#include <stdint.h>
typedef uint32_t ULONG;  /* ThreadX ULONG on the Cortex-M33 port */
typedef int TX_THREAD;
#define TX_TIMER_TICKS_PER_SECOND 1000
static TX_THREAD telemetry_thread, estimator_thread, sensor_thread;
static volatile uint32_t g_telemetry_stack_free_min, g_evaluation_stack_free_min,
    g_distribution_stack_free_min;
static ULONG now;
static unsigned fills[3];
static ULONG tx_time_get(void) { return now; }
static void stack_sample_fill(const TX_THREAD *thread, volatile uint32_t *low) {
  (void)low;
  fills[thread == &telemetry_thread ? 0 : thread == &estimator_thread ? 1 : 2]++;
}
''' + production + r'''
static void run(ULONG first, ULONG ticks) {
  ULONG last = 0; unsigned scans = 0;
  for (ULONG t = 0; t <= ticks; ++t) {
    now = (ULONG)(uint32_t)(first + t);
    for (int pass = 0; pass < 7; ++pass) {   /* several loop passes per tick */
      const uint32_t before = g_telemetry_stack_scans;
      telemetry_sample_stack_margin();
      if (g_telemetry_stack_scans != before) {
        /* Never twice within one second; never more than a second apart
         * while the loop keeps running. */
        if (scans) assert(t - last == TX_TIMER_TICKS_PER_SECOND);
        last = t; ++scans;
      }
    }
  }
  assert(scans == ticks / TX_TIMER_TICKS_PER_SECOND + 1U);
  assert(fills[0] == fills[1] && fills[1] == fills[2]);
}
int main(void) {
  run(0U, 10000U);                    /* the first pass scans at once */
  assert(g_telemetry_stack_scans == 11U && fills[0] == 11U);
  /* The 32-bit ThreadX tick wraps without stalling or bunching scans. */
  g_telemetry_stack_scans = 0U; fills[0] = fills[1] = fills[2] = 0U;
  run(0xFFFFF000UL, 10000U);
  return 0;
}
'''
        compile_and_run(self, harness)

    def test_simulator_low_water_probes_remain(self):
        telemetry = (ROOT / 'Core/Src/telemetry.c').read_text()
        body = telemetry[telemetry.index('static void telemetry_sample_stack_margin'):]
        body = body[:body.index('\n}\n')]
        self.assertIn('tx_time_get()', body)
        for symbol in ('g_telemetry_stack_free_min', 'g_evaluation_stack_free_min',
                       'g_distribution_stack_free_min'):
            self.assertIn(symbol, body)


if __name__ == '__main__':
    unittest.main()
