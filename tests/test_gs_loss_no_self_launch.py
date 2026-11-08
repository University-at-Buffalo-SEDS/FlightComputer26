"""Actual FC timer routine on ground-station loss.

Losing the GS while in the pre-flight aborted state must not send the FC a
Launch signal: that resumed evaluation from an unadmitted internal launch and
requested ignition (valve command 14), which no board acts on. The loss flags
that keep sensor faults from aborting a mission are still set.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def extract(source, signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


STUB = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
typedef uint32_t fu32; typedef uint16_t fu16; typedef unsigned long ULONG;
#define TELEMETRY_ENABLED 1
#define TX_NO_WAIT 0
#define TX_TIMER_TICKS 100
#define GND_TIMEOUT 3000
#define GPS_DELAY_MS 1000
#define TO_ABORT 120
#define option(x) ((x)&~Runtime_Configuration)
#define fc_mask(x) ((x)|FlightComputer_Mask)
#define conditional __attribute__((unused))
/*ENUM*/
enum { HeartbeatGND, HeartbeatRF };
static fu32 g_conf, heartbeat_age, sent;
static int seds_syscall;
static struct { fu16 to_abort, to_reinit; } smon = { TO_ABORT, 0 };
static fu32 timer_fetch(int t) { return t == HeartbeatGND ? heartbeat_age : 0; }
static unsigned tx_queue_send(int *q, fc_msg *m, unsigned w) { (void)q; (void)m; (void)w; ++sent; return 0; }
static void sweetbench_catch(int n) { (void)n; }
static void sweetbench_start(int a, int b) { (void)a; (void)b; }
'''

MAIN = r'''
int main(void) {
  g_conf = option(In_Aborted_State) | option(GPS_Available);
  heartbeat_age = GND_TIMEOUT - 1; fc_timer_routine(0);
  assert(!sent && !(g_conf & option(Lost_GroundStation)));
  heartbeat_age = GND_TIMEOUT + 1;
  for (unsigned i = 0; i < 5; ++i) fc_timer_routine(0);
  assert(!sent && smon.to_abort == TO_ABORT);
  assert(g_conf & option(Lost_GroundStation));
  assert(g_conf & option(Reset_Failures));
  assert(g_conf & option(In_Aborted_State)); /* still awaiting a command */
  return 0;
}
'''


class GroundStationLossTests(unittest.TestCase):
    def test_gs_loss_while_aborted_sends_no_launch_or_ignition(self):
        recovery = (ROOT / 'Core/Src/recovery.c').read_text()
        types = (ROOT / 'Core/Inc/fcstructs.h').read_text()
        enum = types[types.index('typedef enum flight_message : fu32 {'):types.index('} fc_msg;') + len('} fc_msg;')]
        enum = enum.replace('flight_message : fu32', 'flight_message').replace('UINT_FAST32_MAX', 'UINT32_MAX')
        routine = extract(recovery, 'static void fc_timer_routine(')
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / 'probe.c'
            exe = Path(directory) / 'probe'
            src.write_text(STUB.replace('/*ENUM*/', enum) + routine + MAIN)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-fsanitize=address,undefined',
                            str(src), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
