"""Kalman filter status the FC reports to the ground station.

Runs the actual estimator_thread.c status hooks (Launch entry, each completed filter
cycle, a recovery pause) and the control_thread.c stop/reset hooks over a host
fixture, and checks the phase, start stamp and cycle counter that
board_health.c sends in BOARD_HEALTH words 13 and 15.
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
#define fetch_or atomic_fetch_or_explicit
#define load atomic_load_explicit
#define Acq memory_order_acquire
#define Rel memory_order_release
#include "fcconfig.h"
#include "fc_estimator_status.h"
#define STATE_HISTORY_MASK (STATE_HISTORY - 1)
#define option(opt) ((opt) & ~Runtime_Configuration)
#define svec(k) (sv[(((sm.idx) - (k)) & STATE_HISTORY_MASK)])
#define MAX_STATE 8
#define EKF_STATE 6
#define DKF_STATE 4
#define SEDS_DT_ASCENT_LOCAL 1
#define SEDS_DT_ASCENT_STATE 2
#define SEDS_DT_DESCENT_LOCAL 3
#define SEDS_DT_DESCENT_STATE 4
#define TX_SUSPENDED 3
#define id "EV "
/*ACTUAL_ENUM*/
typedef enum { Startup, Postinit, Armed, Launch, Flight_States } state;
typedef struct { float sea, lon, lat; } kf_gps;
typedef struct { kf_gps gps; float alt, vel; } kf_svec;
typedef struct { float q0, rho1, rho2, rho3; } quat;
typedef struct { atomic_uint_fast8_t flight; uint8_t global_state; fu16 idx;
  fi16 confidence; fi16 kf_deviations; } sv_meta;
static _Atomic fu32 g_conf;
static kf_svec sv[STATE_HISTORY];
static sv_meta sm;
static quat qv;
static uint32_t tick, sent, pause_requests;
static bool g_fc_watchdog_resumed;
static struct { int rezero; } baro_conf;
static struct { unsigned tx_thread_state; } estimator_thread = {TX_SUSPENDED};
static uint8_t evaluation_quiescent = 1;
enum { KFRemote, G_Startup };
static struct { unsigned gnd; } rates;
#define now_ms() (tick)
static bool timer_probe_ground(int t, unsigned r) { (void)t; (void)r; return true; }
static void log_periodic_f32(int k, unsigned n, const float *v) { (void)k; (void)n; (void)v; ++sent; }
static void message(const char *m, bool c) { (void)m; (void)c; }
static void ascent_initialize(fu32 c) { (void)c; }
static void distribution_wake(void) {}
static bool fc_recovery_eval_pause_requested(void) { return pause_requests && pause_requests--; }
static void fc_recovery_evaluation_quiescent(void) {}
static void fc_recovery_evaluation_running(void) {}
static void tx_thread_sleep(unsigned t) { (void)t; }
static bool fc_watchdog_startup_reset_ready(void) { return true; }
static void fc_recovery_ota_reset_retire_locked(void) {}
'''

MAIN = r'''
static void cycle(float alt) { sv[sm.idx].alt = alt; sv[sm.idx].vel = 1.0f; tick += 10; propel_kalman_state(g_conf); }
int main(void) {
  g_conf = option(Using_Ascent_KF);
  assert(g_fc_est_phase == FC_EST_IDLE && !g_fc_est_start && !g_fc_est_cycles);
  /* Not started: a preflight monitor lease stops nothing that ran. */
  fc_recovery_control_mark_eval_stopped();
  assert(g_fc_est_phase == FC_EST_IDLE);
  g_conf &= ~option(Eval_Abort_Flag);

  /* Launch resumes the evaluator: initializing until its first cycle. */
  tick = 4990;
  enter_flight_mode(g_conf);
  assert(g_fc_est_phase == FC_EST_INIT && !g_fc_est_start);
  assert(g_conf & option(Launch_Requested));
  cycle(100.0f);
  assert(g_fc_est_phase == FC_EST_RUNNING && g_fc_est_cycles == 1 && sent == 1);
  assert(g_fc_est_start == (5000U | 1U));
  for (unsigned i = 2; i < FC_EST_CONVERGE_CYCLES; ++i) cycle(100.0f);
  assert(g_fc_est_phase == FC_EST_RUNNING);
  cycle(100.0f);
  assert(g_fc_est_phase == FC_EST_CONVERGED && g_fc_est_cycles == FC_EST_CONVERGE_CYCLES);
  assert(g_fc_est_start == 5001U); /* the start stamp holds while it runs */

  /* A recovery pause, then the next cycle runs again. */
  pause_requests = 1;
  evaluation_control_barrier();
  assert(g_fc_est_phase == FC_EST_PAUSED);
  cycle(100.0f);
  assert(g_fc_est_phase == FC_EST_CONVERGED);

  /* A non-finite state, or the FC no longer trusting it, is divergence. */
  cycle(NAN);
  assert(g_fc_est_phase == FC_EST_DIVERGED);
  cycle(INFINITY);
  assert(g_fc_est_phase == FC_EST_DIVERGED);
  sm.kf_deviations = KF_FLUC_THRES + 1;
  cycle(100.0f);
  assert(g_fc_est_phase == FC_EST_DIVERGED);
  sm.kf_deviations = 0;
  const uint32_t cycles = g_fc_est_cycles;

  /* Evaluation Abort / sensor restart stops it ... */
  fc_recovery_control_mark_eval_stopped();
  assert(g_fc_est_phase == FC_EST_STOPPED && (g_conf & option(Eval_Abort_Flag)));
  /* ... and the restart's Launch re-enters: a new start, the counter goes on. */
  tick = 9000;
  enter_flight_mode(g_conf);
  assert(g_fc_est_phase == FC_EST_INIT && !g_fc_est_start);
  cycle(100.0f);
  assert(g_fc_est_phase == FC_EST_RUNNING && g_fc_est_start == (9010U | 1U));
  assert(g_fc_est_cycles == cycles + 1);

  /* A completed Stop/reset back to Startup clears it for the next flight. */
  assert(fc_recovery_control_finalize_startup());
  assert(g_fc_est_phase == FC_EST_IDLE && !g_fc_est_cycles && !g_fc_est_start);
  puts("ok");
  return 0;
}
'''


def build(out):
    evaluation = (BASE / 'Core/Src/estimator_thread.c').read_text()
    recovery = (BASE / 'Core/Src/control_thread.c').read_text()
    types = (BASE / 'Core/Inc/fcstructs.h').read_text()
    enum = types[types.index('typedef enum flight_message : fu32 {'):types.index('} fc_msg;') + len('} fc_msg;')]
    enum = enum.replace('flight_message : fu32', 'flight_message').replace('UINT_FAST32_MAX', 'UINT32_MAX')
    status = evaluation[evaluation.index('/* Estimator status'):evaluation.index('/* FSM helpers */')]
    src = STUB.replace('/*ACTUAL_ENUM*/', enum) + status + '\n' + \
        extract(evaluation, 'static inline void propel_kalman_state(') + '\n' + \
        extract(evaluation, 'static inline void enter_flight_mode(') + '\n' + \
        extract(evaluation, 'static void evaluation_control_barrier(') + '\n' + \
        extract(recovery, 'void fc_recovery_control_mark_eval_stopped(') + '\n' + \
        extract(recovery, 'bool fc_recovery_control_finalize_startup(') + '\n' + MAIN
    (out / 'probe.c').write_text(src)
    return out / 'probe.c'


class EstimatorStatusTests(unittest.TestCase):
    def test_launch_starts_and_each_phase_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            exe = out / 'probe'
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                            '-Wno-unused-parameter', '-fsanitize=address,undefined',
                            '-I', str(BASE / 'Core/Inc'), str(build(out)), '-o', str(exe), '-lm'], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_board_health_carries_the_status_in_words_13_and_15(self):
        source = (BASE / 'Core/Src/board_health.c').read_text()
        self.assertIn('words[13] = FC_EST_WORD13(phase, g_fc_est_cycles);', source)
        self.assertIn('FC_EST_WORD15(g_fc_est_start, baro_ref_bits())', source)
        self.assertIn('words[15] = word15;', source)
        header = (BASE / 'Core/Inc/fc_estimator_status.h').read_text()
        # Wire values the ground station decodes (GroundStation26 fc_estimator.rs).
        for value, name in enumerate(['IDLE', 'INIT', 'RUNNING', 'CONVERGED', 'PAUSED', 'STOPPED', 'DIVERGED']):
            self.assertIn(f'FC_EST_{name}', header)
        self.assertIn('#define FC_EST_TAG 0x4BU', header)


if __name__ == '__main__':
    unittest.main()
