"""The estimator (evaluation thread) runs both filters and the state machine.

Runs the production estimator_thread_entry() loop against a scripted eval_stage:
ascent IMU pair -> predict, barometer -> update + state machine; the switch
to the descent filter inside a pass (Apogee) keeps the descent samples
offered since; a manual switch between passes (Deploy/Expand, with its wake)
drops samples offered to the ascent filter; each descent barometer sample
or GPS fix (outside Vigilant) runs one predict/update and the state machine;
an Evaluation_Abort stops it at its barrier in either phase. The wait times
out after one barometer period: a silent IMU hands the ascent steps to the
barometer alone, a silent barometer gets updates without it (ascent) or
dead-reckoned steps (descent), and every pass checks the drogue timer."""
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
enum { TX_OR=0, TX_AND=2, TX_OR_CLEAR=1, TX_SUCCESS=0, TX_NO_EVENTS=7, TX_WAIT_FOREVER=1, Acq=0, AcqRel=0, AscentKF=0, DescentKF=1 };
typedef enum { EST_NONE, EST_BARO, EST_GPS, EST_SILENT } est_input;
#define EST_STEP_MS   40U
#define EST_SILENT_MS 200U
static uint32_t now, last_imu_ms, last_baro_ms, last_step_ms;
static uint32_t now_ms(void){ return now; }
static struct { bool imu_silent; } plaus;
static void plaus_imu_silent(void *p, bool s){ (void)p; plaus.imu_silent = s; }
static void note_fallback(void){}
static unsigned backups;
static void backup_deployments(void){ ++backups; }
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
/* Script: each wait delivers the next scripted step, ms after the last
 * one; a step that sets nothing is a timeout. */
typedef struct { ULONG set; void (*before)(void); unsigned ms; } step_t;
static step_t script[64]; static unsigned nsteps, next;
static char log_[512]; static unsigned guard;
static void note(const char *s){ strcat(log_, s); strcat(log_, " "); }
static UINT tx_event_flags_set(int *g, ULONG f, UINT op){ (void)g; if (op==TX_AND) group &= f; else group |= f; return 0; }
static UINT tx_event_flags_get(int *g, ULONG req, UINT op, ULONG *got, ULONG w){
  (void)g; (void)op; assert(w == EST_STEP_MS);
  for (;;) {
    if (group & req) { *got = group & req; group &= ~req; return TX_SUCCESS; }
    if (next == nsteps) { g_conf |= Eval_Abort_Flag; *got = 0; return 99; }
    step_t s = script[next++]; now += s.ms ? s.ms : 20U; if (s.before) s.before(); group |= s.set;
    if (!s.set) { *got = 0; return TX_NO_EVENTS; }
  }
}
static unsigned timer_exchange(int t){ (void)t; return 40; }
static void ascent_predict(float dt, fu32 c){ (void)dt; (void)c; assert(guard); note("P"); }
static void ascent_update(void){ assert(guard); note("U"); }
static void descent_predict(float dt){ (void)dt; assert(guard); note("dp"); }
static bool apogee_now;
static void evaluate_rocket_state(fu32 c, float dt){ (void)c; (void)dt; assert(guard); note("E");
  if (apogee_now) { apogee_now = false; g_conf &= ~Using_Ascent_KF; note("APOGEE"); } }
static void distribution_wake(void){}
static void estimator_ascent_update(bool fresh){ if (fresh) ascent_update(); else note("H"); last_baro_ms = fresh ? now : last_baro_ms; }
static void estimator_ascent_baro_only(float *dt){ (void)dt; assert(guard); note("B"); last_baro_ms = now; }
static void estimator_descent_step(fu32 c, float dt, est_input in){
  descent_predict(dt); note(in == EST_SILENT ? "dr" : "du"); if (in == EST_BARO) last_baro_ms = now; evaluate_rocket_state(c, dt); }
static float dt;
/* The control thread parks and terminates a stopped evaluator here. */
static void evaluation_control_barrier(void){ assert(!guard); if (g_conf & Eval_Abort_Flag) longjmp(stopped, 1); }
static void thread_comm_estimator_enter(void){ assert(!guard); guard = 1; }
static void thread_comm_estimator_leave(void){ assert(guard); guard = 0; }
static void enter_flight_mode(fu32 c){ (void)c; }
static void fc_watchdog_checkpoint(void){}
static void at_apogee(void){ apogee_now = true; }
static void manual_swap(void){ g_conf &= ~Using_Ascent_KF; group |= EVAL_STAGE_WAKE; note("MANUAL"); }
/*ENTRY*/
/* Each run starts like a restarted thread: a stale stage from the last one
 * must not survive its entry. */
static void run(void){ memset(log_,0,sizeof log_); next = 0; group = 0; accum = EVALUATION_STAGED; dt = 9.0f; plaus.imu_silent = false;
  if (!setjmp(stopped)) estimator_thread_entry(0); }
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
  /* A silent IMU: barometer bits retained while the estimator waits for an
   * IMU pair; after 200 ms without one it takes the barometer alone; a pair
   * again restores the ordinary steps. A timeout without a silent sensor is
   * no step, but a drogue-timer check. */
  step_t d[] = {{Gyro_Mask|Accl_Mask,0,20},{Baro_Mask,0,20},{0,0,40},{Baro_Mask,0,40},{0,0,40},{Baro_Mask,0,40},
                {0,0,40},{0,0,40},{Baro_Mask,0,40},{Gyro_Mask|Accl_Mask,0,40},{Baro_Mask,0,20}};
  memcpy(script,d,sizeof d); nsteps = sizeof d/sizeof d[0]; g_conf = Using_Ascent_KF; backups = 0; run();
  assert(!strcmp(log_, "P U E B E B E P U E ") && backups >= 8);
  /* A silent barometer in ascent: the staged prediction is updated without
   * it after 200 ms, then on every barometer period. */
  step_t e[] = {{Gyro_Mask|Accl_Mask,0,20},{Baro_Mask,0,20},{Gyro_Mask|Accl_Mask,0,20},{0,0,40},{0,0,40},{0,0,40},
                {0,0,40},{0,0,40},{Gyro_Mask|Accl_Mask,0,20},{0,0,40}};
  memcpy(script,e,sizeof e); nsteps = sizeof e/sizeof e[0]; g_conf = Using_Ascent_KF; run();
  assert(!strcmp(log_, "P U E P H E P H E "));
  /* A silent barometer in descent: a dead-reckoned step every period once
   * 200 ms passed without a sample. */
  step_t f[] = {{Baro_Mask,0,40},{0,0,40},{0,0,40},{0,0,40},{0,0,40},{0,0,40},{0,0,40}};
  memcpy(script,f,sizeof f); nsteps = sizeof f/sizeof f[0]; g_conf = 0; run();
  assert(!strcmp(log_, "dp du E dp dr E dp dr E "));
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
        est = (ROOT / 'Core/Src/estimator_thread.c').read_text()
        entry = '\n'.join(['static fu8 accum;',
                           extract(est, 'ULONG estimator_request(bool ascent)'),
                           extract(est, 'void estimator_left_ascent(void)'),
                           extract(est, 'void estimator_pass(bool ascent, ULONG done)'),
                           extract(est, 'void estimator_thread_entry(ULONG _)')])
        with tempfile.TemporaryDirectory() as tmp:
            src, exe = Path(tmp) / 'e.c', Path(tmp) / 'e'
            src.write_text(HARNESS.replace('/*ENTRY*/', entry))
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-Wno-missing-field-initializers',
                            '-fsanitize=address,undefined', str(src), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
