"""The ground-station and GPS health checks run in the control thread.

They used to be a ThreadX timer (900 ms first, then every 450 ms) executed
inside the SysTick interrupt (TX_TIMER_PROCESS_IN_ISR). The control loop
now runs them on the same grid: health_service() runs every check that is
due, once per slot, and the command wait never blocks past the next slot."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def extract(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t ULONG; typedef int32_t LONG;
#define TX_NO_WAIT 0U
#define HEALTH_CHECK_TICKS 450U
#define HEALTH_CHECK_FIRST_TICKS (HEALTH_CHECK_TICKS * 2)
static ULONG now = 0xFFFFFE00U; /* across the 32-bit tick wrap */
static ULONG checks[16]; static unsigned nchecks;
static ULONG tx_time_get(void) { return now; }
static void health_check(void) { checks[nchecks++] = now; }
/*CODE*/
int main(void)
{
  ULONG start = now;
  health_service(); assert(nchecks == 0);           /* not started */
  health_start();
  assert(health_wait(250) == 250);
  now = start + 899; health_service(); assert(nchecks == 0);
  assert(health_wait(250) == 1);
  now = start + 900; assert(health_wait(250) == TX_NO_WAIT);
  health_service(); assert(nchecks == 1 && checks[0] == start + 900);
  assert(health_wait(250) == 250);
  now = start + 900 + 449; assert(health_wait(250) == 1);
  /* A command held the thread past two slots: both checks run, the grid
   * is kept (the next one is still at 900 + 3 * 450). */
  now = start + 900 + 950; health_service();
  assert(nchecks == 3);
  assert(health_wait(500) == start + 900 + 3 * 450 - now);
  return 0;
}
'''


class HealthSchedule(unittest.TestCase):
    def test_same_grid_as_the_timer(self):
        recovery = (ROOT / 'Core/Src/recovery.c').read_text()
        code = '\n'.join(re.sub(r'^static ', 'static ', l) for l in
                         [recovery[recovery.index('static bool health_active;'):recovery.index('/* Task */', recovery.index('static bool health_active;'))]])
        with tempfile.TemporaryDirectory() as tmp:
            src, exe = Path(tmp) / 'h.c', Path(tmp) / 'h'
            src.write_text(HARNESS.replace('/*CODE*/', code))
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            str(src), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_loop_runs_checks_and_never_waits_past_them(self):
        recovery = (ROOT / 'Core/Src/recovery.c').read_text()
        self.assertNotIn('tx_timer_create', recovery)
        self.assertNotIn('monotonic_checks', recovery)
        entry = recovery[recovery.index('void recovery_entry(ULONG st)'):]
        entry = entry[:entry.index('\n}\n')]
        loop = entry[entry.index('MrAnalog (WE_ARE_SO_BACK)'):]
        self.assertLess(entry.index('health_start();'), entry.index('MrAnalog (WE_ARE_SO_BACK)'))
        self.assertLess(loop.index('health_service();'), loop.index('fc_recovery_control_service();'))
        self.assertIn('thread_comm_receive(&msg, health_wait(TX_TIMER_TICKS_PER_SECOND / 4U))', loop)
        config = (ROOT / 'Core/Inc/fcconfig.h').read_text()
        self.assertRegex(config, r'#define HEALTH_CHECK_TICKS\s+450\n')
        self.assertIn('#define HEALTH_CHECK_FIRST_TICKS (HEALTH_CHECK_TICKS * 2)', config)


if __name__ == '__main__':
    unittest.main()
