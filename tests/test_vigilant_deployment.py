"""Actual evaluator FSM and deployment gates over a simulated flight.

The vigilant watchdog is enabled by default (USER_OPTIONS). It must correct the
flight state without suppressing the drogue or main deployment.
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
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint_fast8_t fu8; typedef uint_fast16_t fu16; typedef uint32_t fu32;
typedef int16_t fi16; typedef int32_t fi32;
#define conditional __attribute__((unused))
#define fetch_add atomic_fetch_add_explicit
#define fetch_or atomic_fetch_or_explicit
#define store atomic_store_explicit
#define load atomic_load_explicit
#define Acq memory_order_acquire
#define Rel memory_order_release
#define AcqRel memory_order_acq_rel
#include "fcconfig.h"
#define STATE_HISTORY_MASK (STATE_HISTORY - 1)
#define maxd(a, b) ((a) > (b) ? (a) : (b))
#define mind(a, b) ((a) < (b) ? (a) : (b))
#define option(opt) ((opt) & ~Runtime_Configuration)
#define within(expr, bound) (fabsf((float)(expr)) <= (bound))
#define svec(k) (sv[(((sm.idx) - (k)) & STATE_HISTORY_MASK)])
#define Require(cond) do { if (!(cond)) { handle_spurious(mode, #cond); return; } } while (0)
#define MAX_SPURIOUS_REPORT_SIZE 96
#define MAX_METRIC_REPORT_SIZE 96
#define id "EV "
#define sweetbench_start(...) ((void)0)
#define fc_sd_capture_event(e, r) ((void)0)
#define BOARD_WATCHDOG_ENABLE 0
/*ACTUAL_ENUM*/
typedef enum { Startup, Postinit, Armed, Launch, Ascent, Coast, Apogee,
  Descent, Reefing, Landed, Recovery, Flight_States } state;
typedef struct { float sea, lon, lat; } kf_gps;
typedef struct { float alt, prs, tmp; } baro;
typedef struct { kf_gps gps; float alt, vel; float bias[4]; } kf_svec;
typedef struct { atomic_uint_fast8_t flight; uint8_t global_state; fu16 idx;
  fi16 confidence; fi16 kf_deviations; } sv_meta;
typedef struct { float max_alt, max_vel; } stats;
static struct { baro baro; kf_gps gps; } meas;
static _Atomic fu32 g_conf;
static kf_svec sv[STATE_HISTORY];
static sv_meta sm;
static stats extremes;
static float kalt, kvel;
static uint32_t sim_ms, aux_ms, skip_ms;
static unsigned drogue, mains; static float drogue_alt, main_alt;
enum { Auxiliary };
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t m) { (void)m; }
static void fc_recovery_ota_reset_retire_locked(void) {}
static void message(const char *m, bool c) { (void)m; (void)c; }
static void log_metric(const char *m, fi32 v, bool c) { (void)m; (void)v; (void)c; }
static void sd_launch_landed(uint32_t t) { (void)t; }
static uint32_t now_ms(void) { return sim_ms; }
static int *to_global_state(state s) { static int g; g = s; return &g; }
static void log_flight_state(int *s) { (void)s; }
static void timer_update(int t) { (void)t; aux_ms = sim_ms; }
static uint32_t timer_fetch(int t) { (void)t; return sim_ms - aux_ms; }
static void tx_thread_sleep(uint32_t ticks) { skip_ms += ticks; }
static void descent_initialize(fu32 c) { (void)c; }
static state current(void) { return load(&sm.flight, Acq); }
static bool beyond(state b) { return current() > b; }
static bool fc_watchdog_can_actuate(void) { return true; }
static void deployment_pulse_start(bool reef) {
  if (reef) { mains++; main_alt = svec(0).alt; g_conf |= option(Parachute_Expanded); }
  else { drogue++; drogue_alt = svec(0).alt; g_conf |= option(Parachute_Deployed); }
}
'''

MAIN = r'''
static void fc_watchdog_checkpoint(void) {}
static void board_watchdog_progress(unsigned m) { (void)m; }
static void propel_kalman_state(fu32 c) { (void)c; sm.idx = (sm.idx + 1) & STATE_HISTORY_MASK; }
#define BOARD_WATCHDOG_SAFETY 0
/*ACTUAL_EVALUATE*/
int main(int argc, char **argv) {
  assert(argc == 2);
  const bool coarse = !strcmp(argv[1], "coarse");
  const fu32 conf = strcmp(argv[1], "novigilant") ? USER_OPTIONS
                  : (USER_OPTIONS & ~option(Vigilant_Mode));
  g_conf = conf;
  store(&sm.flight, Armed, Rel);
  const float dt = coarse ? 0.5f : 0.02f, g = 9.80665f;
  float alt = 0.0f, vel = 0.0f, apogee = 0.0f, apogee_vel = 0.0f;
  state reached = Armed;
  for (unsigned step = 0; step < 200000; ++step) {
    const float t = step * dt;
    float acc = t < 3.5f ? 50.0f : -g;
    vel += acc * dt;
    if (t >= 3.5f && (g_conf & option(Parachute_Expanded))) vel = fmaxf(vel, -6.0f);
    else if (t >= 3.5f && (g_conf & option(Parachute_Deployed))) vel = fmaxf(vel, -25.0f);
    alt += vel * dt;
    apogee = fmaxf(apogee, alt);
    sim_ms = (uint32_t)(t * 1000.0f);
    if (alt <= 0.0f && t > 5.0f) break;
    sv[sm.idx].alt = alt; sv[sm.idx].vel = vel; meas.baro.alt = alt;
    if (skip_ms) { const uint32_t step_ms = (uint32_t)(dt * 1000.0f); skip_ms = skip_ms > step_ms ? skip_ms - step_ms : 0U; continue; }
    evaluate_rocket_state(g_conf, dt);
    if (current() == Apogee && reached < Apogee) apogee_vel = vel;
    if (current() > reached) reached = current();
  }
  printf("%s: apogee=%.0f reached=%u drogue=%u@%.0f main=%u@%.0f\n", argv[1], apogee,
         (unsigned)reached, drogue, drogue_alt, mains, main_alt); fflush(stdout);
  assert(reached >= Reefing);
  assert(drogue == 1 && drogue_alt > apogee - 150.0f);
  if (coarse) assert(apogee_vel < -APOGEE_MAX_VEL); /* outside detect_apogee's window */
  assert(mains == 1 && main_alt <= REEF_TARGET_ALT && main_alt > 300.0f);
  return 0;
}
'''


def build(out):
    evaluation = (BASE / 'Core/Src/evaluation.c').read_text()
    fcapi = (BASE / 'Core/Inc/fcapi.h').read_text()
    types = (BASE / 'Core/Inc/fcstructs.h').read_text()
    enum = types[types.index('typedef enum flight_message : fu32 {'):types.index('} fc_msg;') + len('} fc_msg;')]
    enum = enum.replace('flight_message : fu32', 'flight_message').replace('UINT_FAST32_MAX', 'UINT32_MAX')
    fsm = evaluation[evaluation.index('/* FSM helpers */'):evaluation.index('/* FSM */')]
    pyro = extract(fcapi, 'static inline bool release_parachute(') + '\n' + \
        extract(fcapi, 'static inline bool expand_parachute(')
    src = STUB.replace('/*ACTUAL_ENUM*/', enum) + pyro + '\n' + fsm + \
        MAIN.replace('/*ACTUAL_EVALUATE*/', extract(evaluation, 'void evaluate_rocket_state('))
    (out / 'probe.c').write_text(src)
    return out / 'probe.c'


class VigilantDeploymentTests(unittest.TestCase):
    def run_flight(self, case):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            exe = out / 'probe'
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function', '-Wno-unused-parameter',
                            '-fsanitize=address,undefined', '-I', str(BASE / 'Core/Inc'),
                            str(build(out)), '-o', str(exe), '-lm'], check=True)
            return subprocess.run([str(exe), case], capture_output=True, text=True, timeout=30)

    def test_nominal_flight_without_vigilant_fires_drogue_and_main(self):
        result = self.run_flight('novigilant')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_vigilant_correction_after_missed_apogee_fires_drogue_and_main(self):
        # 2 Hz samples step over the +/-4 m/s apogee window, so only the
        # vigilant watchdog can move the FSM out of Coast.
        result = self.run_flight('coarse')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_default_vigilant_mode_still_fires_drogue_and_main(self):
        result = self.run_flight('default')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
