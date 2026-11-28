"""Operator Kalman filter commands (docs/kalman-commands.md).

Kalman stop/reset/start are fc_msg Actionable_Decrees + 22..24 and one-byte
compat 34..36. Their owner/control semantics (a false detection undone, a
station Launch never stoppable, refusals) run against the actual recovery
owner in test_admitted_flight.py (kf_* cases). Here: the wire values, and
that the estimator cancel the stop and reset use clears exactly the
estimator's flight memory and keeps the pad's calibration."""
from pathlib import Path
import re
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


HARNESS = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "rud_detect.h"
#include "sensor_plausibility.h"
#include "deploy_backup.h"
#define STATE_HISTORY 8U
typedef uint8_t fu8;
typedef struct { float sea, lon, lat; } kf_gps;
typedef struct { float az, gx, gy, gz; } ekf_bias;
typedef struct { kf_gps gps; float alt, vel; ekf_bias bias; } kf_svec;
typedef struct { float max_alt, max_vel; } stats;
static kf_svec sv[STATE_HISTORY];
static struct { unsigned flight, global_state, idx, confidence, kf_deviations; } sm;
static stats extremes;
static uint8_t backup_armed;
static rud_state rud;
static plaus_state plaus;
static backup_dr dr;
static volatile uint8_t g_fc_sensor_fallback;
static uint32_t last_imu_ms, last_baro_ms, last_step_ms, est_run;
static float dt;
static fu8 accum;
static struct { float alt, vel; bool have, baro_was_ok; } imu_track;
static struct { float sea; uint32_t moved_ms; bool seen; } gps_watch;
static bool staged = true;
static void thread_comm_ascent_stage(bool s) { staged = s; }
static uint32_t now_ms(void) { return 4242U; }
/*ACTUAL*/
int main(void)
{
  for (unsigned k = 0; k < STATE_HISTORY; ++k) {
    sv[k].gps = (kf_gps){1.0f, 2.0f, 3.0f};
    sv[k].alt = 900.0f; sv[k].vel = 80.0f;
    sv[k].bias = (ekf_bias){0.1f, 0.2f, 0.3f, 0.4f};
  }
  sm.flight = 4; sm.global_state = 16; sm.idx = 5;
  extremes = (stats){1200.0f, 150.0f};
  backup_armed = 1; rud.fired = true; dr.active = true; dr.drogue = true;
  memset(&plaus, 0x5a, sizeof plaus);
  g_fc_sensor_fallback = 9; est_run = 77; dt = 0.04f; accum = 3;
  imu_track.have = true; imu_track.baro_was_ok = false; gps_watch.seen = true;
  estimator_cancel_evaluation();
  for (unsigned k = 0; k < STATE_HISTORY; ++k) {
    assert(sv[k].gps.sea == 0.0f && sv[k].gps.lon == 0.0f && sv[k].gps.lat == 0.0f);
    assert(sv[k].alt == 0.0f && sv[k].vel == 0.0f);
    /* The pad's calibration stays. */
    assert(sv[k].bias.az == 0.1f && sv[k].bias.gz == 0.4f);
  }
  /* Exactly the estimator's memory: the flight state is not its to touch. */
  assert(sm.flight == 4 && sm.global_state == 16 && sm.idx == 0);
  assert(extremes.max_alt == 0.0f && extremes.max_vel == 0.0f);
  /* The drogue timer is disarmed and the break-up detector cleared. */
  assert(!backup_armed && !dr.active && !dr.drogue && !rud.fired);
  plaus_state clean; memset(&clean, 0, sizeof clean);
  assert(!memcmp(&plaus, &clean, sizeof plaus));
  assert(!g_fc_sensor_fallback && !est_run && dt == 0.0f && !accum && !staged);
  assert(!imu_track.have && imu_track.baro_was_ok && !gps_watch.seen);
  assert(last_imu_ms == 4242U && last_baro_ms == 4242U && last_step_ms == 4242U);
  return 0;
}
'''


class KalmanCommands(unittest.TestCase):
    def setUp(self):
        self.types = (ROOT / 'Core/Inc/fcstructs.h').read_text()
        self.dist = (ROOT / 'Core/Src/sensor_thread.c').read_text()
        self.recovery = (ROOT / 'Core/Src/control_thread.c').read_text()

    def test_wire_values_follow_the_last_decree_and_byte(self):
        self.assertIn('Kalman_Stop      = Actionable_Decrees + 22,', self.types)
        self.assertIn('Kalman_Reset     = Actionable_Decrees + 23,', self.types)
        self.assertIn('Kalman_Start     = Actionable_Decrees + 24,', self.types)
        compat = self.types[self.types.index('typedef enum remote_cmd_compat'):self.types.index('} compat;')]
        names = re.findall(r'\n\s*(\w+),', compat)
        self.assertEqual(names[34:], ['Compat_Kalman_Stop', 'Compat_Kalman_Reset', 'Compat_Kalman_Start'])
        table = self.dist[self.dist.index('static const fc_msg extmap[Compat_Messages] = {'):]
        table = table[table.index('{') + 1:table.index('};')]
        entries = [e.strip() for e in re.sub(r'/\*.*?\*/', '', table, flags=re.S).split(',') if e.strip()]
        self.assertEqual(entries[33:], ['SD_Retry', 'Kalman_Stop', 'Kalman_Reset', 'Kalman_Start'])

    def test_only_the_operator_sends_them(self):
        body = extract(self.recovery, 'static inline void process_action(')
        for name, handler in (('Kalman_Stop', 'kalman_stop'), ('Kalman_Reset', 'kalman_reset'),
                              ('Kalman_Start', 'kalman_start')):
            case = body[body.index('case %s:' % name):]
            self.assertTrue(case.split('break;')[0].strip().endswith('if (!internal) %s();' % handler), name)

    def test_stop_never_publishes_or_resets_the_global_state(self):
        hook = extract(self.recovery, 'void fc_recovery_control_kf_stopped(')
        for forbidden in ('log_flight_state', 'G_Startup', 'sm.flight = Startup', 'advance_epoch',
                          'Parachute_Deployed', 'fc_watchdog_hold_for_operator_reset'):
            self.assertNotIn(forbidden, hook)
        self.assertIn('fc_cancel_evaluation();', hook)
        self.assertIn('fc_watchdog_discard_flight();', hook)

    def test_cancel_clears_exactly_the_estimator_memory(self):
        evaluation = (ROOT / 'Core/Src/estimator_thread.c').read_text()
        actual = extract(evaluation, 'static inline void sensor_verdicts_reset(') + '\n' + \
            extract(evaluation, 'void estimator_cancel_evaluation(')
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            src = out / 'cancel.c'
            src.write_text(HARNESS.replace('/*ACTUAL*/', actual))
            exe = out / 'cancel'
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined',
                            '-I', str(ROOT / 'Core/Inc'), str(src),
                            str(ROOT / 'Core/Src/rud_detect.c'), str(ROOT / 'Core/Src/sensor_plausibility.c'),
                            str(ROOT / 'Core/Src/deploy_backup.c'), '-lm', '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
