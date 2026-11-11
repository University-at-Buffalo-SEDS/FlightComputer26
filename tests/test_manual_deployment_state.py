"""The operator's manual Deploy/Expand moves the FC flight state only when the pulse is accepted.

A refused release_parachute (state not beyond Armed, already deployed, watchdog
or inhibit) must leave the flight state untouched, as a refused Expand already
does; otherwise a premature or repeated Deploy would move the FSM to Descent
and disturb the apogee and drogue logic. The actual manual_deployment from
recovery.c is compiled against stubs of its callees.
"""
from pathlib import Path
import subprocess
import tempfile
import unittest

BASE = Path(__file__).resolve().parents[1]


def extract(s, signature):
    start = s.index(signature)
    end = s.index('{', start) + 1
    depth = 1
    while depth:
        depth += (s[end] == '{') - (s[end] == '}')
        end += 1
    return s[start:end]


STUB = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef int32_t fi32;
#define TELEMETRY_ENABLED 1
#define id "RE "
enum { Startup, Postinit, Armed, Launch, Ascent, Coast, Apogee, Descent, Reefing, Landed, Recovery };
#define Runtime_Configuration (1u << 31)
#define Using_Ascent_KF (Runtime_Configuration | (1u << 21))
#define option(opt) ((opt) & ~Runtime_Configuration)
#define FC_LOG_PYRO_REJECTED 7
static struct { unsigned flight; } sm;
static uint32_t g_conf;
static bool release_ok, expand_ok, inhibited;
static int released, expanded, rejected_events, underglow, reports;
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t m) { (void)m; }
static void fc_recovery_ota_reset_retire_locked(void) {}
static bool fc_recovery_inhibits_manual_pyro(void) { return inhibited; }
static bool fc_recovery_inhibits_pyro(void) { return inhibited; }
static bool fc_recovery_reset_any_inflight(void) { return false; }
static bool fc_watchdog_can_actuate(void) { return true; }
static void fc_sd_capture_event(int e, bool reef) { (void)e; (void)reef; rejected_events++; }
static bool release_parachute(bool force, bool manual) { (void)force; (void)manual; released++; return release_ok; }
static bool expand_parachute(bool force, bool manual) { (void)force; (void)manual; expanded++; return expand_ok; }
static void av_bay_underglow_signal(unsigned v) { underglow = (int)v; }
static void log_metric(const char *m, fi32 v, bool c) { (void)m; (void)v; (void)c; reports++; }
static void message(const char *m, bool c) { (void)m; (void)c; reports++; }
static unsigned to_global_state(unsigned s) { return s; }
static void log_flight_state(unsigned s) { (void)s; reports++; }
static void descent_initialize(uint32_t c) { (void)c; }
/*MANUAL_DEPLOYMENT*/
static void reset(unsigned state, bool rel, bool exp) {
  sm.flight = state; release_ok = rel; expand_ok = exp; inhibited = false;
  released = expanded = rejected_events = underglow = reports = 0;
}
int main(void) {
  /* A refused manual Deploy (here at Armed) leaves the state where it was. */
  reset(Armed, false, false); manual_deployment(true, false, true);
  assert(released == 1); assert(sm.flight == Armed); assert(underglow == 0);
  /* A refused repeat Deploy in Coast does not jump the FSM to Descent. */
  reset(Coast, false, false); manual_deployment(true, false, true);
  assert(sm.flight == Coast);
  /* An accepted Deploy moves to Descent and signals it. */
  reset(Coast, true, false); manual_deployment(true, false, true);
  assert(sm.flight == Descent); assert(underglow == 2);
  /* Expand: refused keeps the state, accepted moves to Reefing. */
  reset(Descent, false, false); manual_deployment(false, false, true);
  assert(sm.flight == Descent); assert(underglow == 0);
  reset(Descent, false, true); manual_deployment(false, false, true);
  assert(sm.flight == Reefing); assert(underglow == 4);
  /* An inhibited manual pyro never reaches the pulse. */
  reset(Coast, true, true); inhibited = true; manual_deployment(true, false, true);
  assert(released == 0 && sm.flight == Coast && rejected_events == 1);
  puts("manual deployment state OK");
  return 0;
}
'''


class ManualDeploymentStateTests(unittest.TestCase):
    def test_refused_manual_deploy_leaves_flight_state(self):
        recovery = (BASE / 'Core/Src/recovery.c').read_text()
        src = STUB.replace('/*MANUAL_DEPLOYMENT*/', extract(recovery, 'static inline void manual_deployment('))
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            (out / 'probe.c').write_text(src)
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            '-fsanitize=address,undefined', str(out / 'probe.c'), '-o', str(out / 'probe')], check=True)
            result = subprocess.run([str(out / 'probe')], capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_operator_deploy_and_expand_are_forced_in_any_state(self):
        # Emergency chutes: the operator's Deploy/Expand (not internal) is
        # forced past the flight-state checks and every FC hold but a wired
        # bench session; automatic deployments keep the state checks.
        recovery = (BASE / 'Core/Src/recovery.c').read_text()
        action = extract(recovery, 'static inline void process_action(')
        plain = action[action.index('#else'):action.index('#endif /* LUNATIC_STATE */')]
        self.assertIn('manual_deployment(true, !internal, !internal)', plain)
        self.assertIn('manual_deployment(false, !internal, !internal)', plain)
        admissible = extract(recovery, 'bool fc_recovery_command_admissible(')
        self.assertLess(admissible.index('Deploy_Parachute'), admissible.index('fc_recovery_holds_sequence()'))
        owner = (BASE / 'Core/Src/fc_recovery_owner.c').read_text()
        self.assertIn('return owner.monitor_mode || reset_any;', extract(owner, 'bool fc_recovery_inhibits_manual_pyro('))  # op 3 window only
        distribution = (BASE / 'Core/Src/distribution.c').read_text()
        self.assertIn('board_recovery_operator_pyro_allowed(pkt)', distribution)


if __name__ == '__main__':
    unittest.main()
