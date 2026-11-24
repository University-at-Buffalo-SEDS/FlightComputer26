"""The estimator (evaluation thread) runs both filters and the state machine.

Runs the production estimator_thread_entry() loop against a scripted eval_stage:
ascent IMU pair -> predict, barometer -> update + state machine; the switch
to the descent filter inside a pass (Apogee) keeps the descent samples
offered since; a manual switch between passes (Deploy/Expand, with its wake)
drops samples offered to the ascent filter; each descent barometer sample
or GPS fix (outside Vigilant) runs one predict/update and the state machine;
an Evaluation_Abort stops it at its barrier in either phase."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <setjmp.h>
static jmp_buf stopped;
typedef unsigned long ULONG; typedef unsigned UINT; typedef uint8_t fu8; typedef uint32_t fu32;
enum { TX_OR=0, TX_AND=2, TX_OR_CLEAR=1, TX_SUCCESS=0, TX_WAIT_FOREVER=1, Acq=0, AcqRel=0, AscentKF=0, DescentKF=1 };
enum { Baro_Mask=1, Gyro_Mask=2, Accl_Mask=4 };
#define EVALUATION_STAGED (fu8)(1u << 7)
#define EVAL_STAGE_GPS  (1UL << 8)
#define EVAL_STAGE_WAKE (1UL << 9)
enum { Using_Ascent_KF=1u<<0, Ascent_KF_Staged=1u<<1, Eval_Abort_Flag=1u<<2 };
#define option(x) (x)
#define load(p,o) (*(p))
static fu32 fetch_and(fu32 *p, fu32 v, int o){ (void)o; fu32 r=*p; *p&=v; return r; }
static fu32 fetch_or(fu32 *p, fu32 v, int o){ (void)o; fu32 r=*p; *p|=v; return r; }
#define fsec(x) ((float)(x))
#define sweetbench_catch(x) ((void)0)
#define sweetbench_start(...) ((void)0)
static fu32 g_conf = Using_Ascent_KF;
static int eval_stage;
static ULONG group;
/* Script: each wait delivers the next scripted step. */
typedef struct { ULONG set; void (*before)(void); } step_t;
static step_t script[64]; static unsigned nsteps, next;
static char log_[512]; static unsigned guard;
static void note(const char *s){ strcat(log_, s); strcat(log_, " "); }
static UINT tx_event_flags_set(int *g, ULONG f, UINT op){ (void)g; if (op==TX_AND) group &= f; else group |= f; return 0; }
static UINT tx_event_flags_get(int *g, ULONG req, UINT op, ULONG *got, ULONG w){
  (void)g; (void)op; (void)w;
  for (;;) {
    if (group & req) { *got = group & req; group &= ~req; return TX_SUCCESS; }
    if (next == nsteps) { g_conf |= Eval_Abort_Flag; *got = 0; return 7; }
    step_t s = script[next++]; if (s.before) s.before(); group |= s.set;
  }
}
static unsigned timer_exchange(int t){ (void)t; return 40; }
static void ascent_predict(float dt, fu32 c){ (void)dt; (void)c; assert(guard); note("P"); }
static void ascent_update(void){ assert(guard); note("U"); }
static void descent_predict(float dt){ (void)dt; assert(guard); note("dp"); }
static void descent_update(void){ assert(guard); note("du"); }
static bool apogee_now;
static void evaluate_rocket_state(fu32 c, float dt){ (void)c; (void)dt; assert(guard); note("E");
  if (apogee_now) { apogee_now = false; g_conf &= ~Using_Ascent_KF; note("APOGEE"); } }
static void distribution_wake(void){}
static void estimator_ascent_update(void){ ascent_update(); }
static void estimator_descent_step(fu32 c, float dt){ descent_predict(dt); descent_update(); evaluate_rocket_state(c, dt); }
/* The control thread parks and terminates a stopped evaluator here. */
static void evaluation_control_barrier(void){ assert(!guard); if (g_conf & Eval_Abort_Flag) longjmp(stopped, 1); }
static void thread_comm_estimator_enter(void){ assert(!guard); guard = 1; }
static void thread_comm_estimator_leave(void){ assert(guard); guard = 0; }
static void enter_flight_mode(fu32 c){ (void)c; }
static void fc_watchdog_checkpoint(void){}
static void at_apogee(void){ apogee_now = true; }
static void manual_swap(void){ g_conf &= ~Using_Ascent_KF; group |= EVAL_STAGE_WAKE; note("MANUAL"); }
/*ENTRY*/
static void run(void){ memset(log_,0,sizeof log_); next = 0; group = 0; if (!setjmp(stopped)) estimator_thread_entry(0); }
int main(void){
  /* Ascent pair/baro cycles, then Apogee in a pass; descent samples offered
   * during and after it are stepped. */
  step_t a[] = {{Gyro_Mask,0},{Accl_Mask,0},{Baro_Mask,at_apogee},{Gyro_Mask|Accl_Mask,0},{Baro_Mask,0},
                {Baro_Mask,0},{EVAL_STAGE_GPS,0},{Gyro_Mask,0},{Baro_Mask|EVAL_STAGE_GPS,0}};
  memcpy(script,a,sizeof a); nsteps = sizeof a/sizeof a[0]; g_conf = Using_Ascent_KF; run();
  assert(!strcmp(log_, "P U E APOGEE dp du E dp du E dp du E dp du E "));
  /* A retained ascent barometer bit before a manual switch is not a
   * descent step; the wake makes the estimator change mode at once. */
  step_t b[] = {{Gyro_Mask|Accl_Mask,0},{Baro_Mask,manual_swap},{Baro_Mask,0}};
  memcpy(script,b,sizeof b); nsteps = 3; g_conf = Using_Ascent_KF; run();
  assert(!strcmp(log_, "P MANUAL dp du E "));
  /* Evaluation_Abort ends the loop in descent too. */
  step_t c[] = {{Baro_Mask,0}};
  memcpy(script,c,sizeof c); nsteps = 1; g_conf = 0; run();
  assert(!strcmp(log_, "dp du E ") && (g_conf & Eval_Abort_Flag));
  return 0;
}
'''


def extract(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


class EstimatorLoop(unittest.TestCase):
    def test_both_filters_and_mode_switches(self):
        entry = extract((ROOT / 'Core/Src/estimator_thread.c').read_text(), 'void estimator_thread_entry(ULONG _)')
        with tempfile.TemporaryDirectory() as tmp:
            src, exe = Path(tmp) / 'e.c', Path(tmp) / 'e'
            src.write_text(HARNESS.replace('/*ENTRY*/', entry))
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                            '-fsanitize=address,undefined', str(src), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
