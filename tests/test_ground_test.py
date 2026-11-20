"""Integrated ground test (Core/Src/ground_test.c, docs in GroundStation26
docs/integrated-ground-test.md).

Compiles the actual ground_test.c against a host port whose "SD card" is a
temporary directory, and the actual deployment_pulse.c pyro gate and the
distribution.c GPS wrapper against small stubs. Covers CSV parsing and
validation, interpolation, end of data, the mode gates (replay only after a
validated pre-Launch entry and at rest), dry-run versus LIVE PYRO, Stop/reset
back to Off, the GPS switch to real at landing, and that a flight image
cannot reach any of it.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CC = ['cc', '-std=gnu11', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
      '-Wno-unused-function', '-Wno-unused-parameter',
      '-fsanitize=address,undefined', '-fno-sanitize-recover=all']

PORT = r'''
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "fcconfig.h"
#define IGT_PUB_PERIOD_MS 200U
#define IGT_SD_PERIOD_MS 50U
char igt_root[512];
int igt_fail_sd;
static FILE *igt_fp;
char igt_log[64][96];
unsigned igt_logs, igt_samples[8], igt_radio, igt_sd;
float igt_last_sample[8][8];
static int igt_port_open(const char *path)
{
  if (igt_fail_sd) return -2;
  char full[640];
  snprintf(full, sizeof full, "%s%s", igt_root, path);
  igt_fp = fopen(full, "rb");
  return igt_fp ? 0 : -1;
}
static long igt_port_read(uint32_t off, char *buf, uint32_t cap)
{
  if (fseek(igt_fp, (long)off, SEEK_SET) != 0) return -1;
  return (long)fread(buf, 1, cap, igt_fp);
}
static void igt_port_close(void) { fclose(igt_fp); igt_fp = NULL; }
static void igt_port_message(const char *text)
{
  if (igt_logs < 64) snprintf(igt_log[igt_logs++], 96, "%s", text);
}
static void igt_port_sample(const float v[8], bool radio, bool sd)
{
  const unsigned kind = (unsigned)v[0];
  if (kind < 8) { ++igt_samples[kind]; memcpy(igt_last_sample[kind], v, 8 * sizeof(float)); }
  igt_radio += radio; igt_sd += sd;
}
'''

DRIVER = r'''
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include "ground_test.c"

static const float rest_a[3] = {0.0f, 0.0f, 9.80665f};
static const float rest_g[3] = {0.1f, -0.2f, 0.05f};

static bool logged(const char *needle)
{
  for (unsigned i = 0; i < igt_logs; ++i) if (strstr(igt_log[i], needle)) return true;
  return false;
}
static unsigned mode_now(void) { return (igt_health_bits() >> 4) & 7U; }
static unsigned reason_now(void) { return (igt_health_bits() >> 12) & 15U; }
static void enter_and_validate(void)
{
  assert(igt_command(IGT_CMD_ENTER, true, false));
  assert(mode_now() == IGT_VALIDATING);
  igt_sd_service();
}
static bool near(float a, float b) { return fabsf(a - b) < 1e-3f; }

/* Real sensors at rest, replay at t: one 10 ms cycle like the distribution task */
static void cycle(uint32_t now, float a[3], float g[3], float b[3], float real_alt)
{
  memcpy(a, rest_a, sizeof rest_a); memcpy(g, rest_g, sizeof rest_g);
  b[0] = real_alt; b[1] = 101000.0f; b[2] = 20.0f;
  igt_override_accl(a, now); igt_override_gyro(g, now); igt_override_baro(b, now);
  if (now % 50U == 0U) igt_sd_service();
}

static void scenario_validate(void)
{
  enter_and_validate();
  printf("mode=%u reason=%u gps=%u imu=%u baro=%u\n", mode_now(), reason_now(),
         (unsigned)gps_present, (unsigned)ring_count(&streams[S_IMU]),
         (unsigned)ring_count(&streams[S_BARO]));
  for (unsigned i = 0; i < igt_logs; ++i) printf("log: %s\n", igt_log[i]);
}

static void scenario_numbers(void)
{
  float v;
  assert(parse_number("12.5", &v, false) && near(v, 12.5f));
  assert(parse_number("-0.031", &v, false) && near(v, -0.031f));
  assert(parse_number("+3", &v, false) && near(v, 3.0f));
  assert(parse_number("31.7580123", &v, false) && fabsf(v - 31.758012f) < 1e-5f);
  assert(parse_number("1.23456789999", &v, false) && near(v, 1.2345679f));
  assert(!parse_number("1234567890", &v, false));   /* integer part too long */
  assert(!parse_number(".", &v, false));
  assert(!parse_number("-5", &v, true));            /* times are unsigned */
  assert(!parse_number("x1", &v, false));
  const char *end = parse_number("7e3", &v, false);  /* no exponent: stops */
  assert(end && *end == 'e');
  igt_row row;
  assert(parse_row(&streams[S_BARO], "10,1.5,101000,20", &row) == 0 && row.t == 10);
  assert(parse_row(&streams[S_BARO], "10,1.5,101000", &row) == IGT_E_PARSE);
  assert(parse_row(&streams[S_BARO], "10,1.5,101000,20,9", &row) == IGT_E_PARSE);
  assert(parse_row(&streams[S_BARO], "10,1.5,101000,20 ", &row) == IGT_E_PARSE);
  assert(parse_row(&streams[S_BARO], "10,1.5,5000,20", &row) == IGT_E_RANGE);
  assert(parse_row(&streams[S_IMU], "0,0,0,9.8,0,0,99999", &row) == IGT_E_RANGE);
  printf("numbers ok\n");
}

/* The flight: ringed replay, interpolation, motion of the replay clock,
 * GPS at 10 Hz, SIM_SAMPLE publication and the switch to real at landing. */
static void scenario_flight(void)
{
  float a[3], g[3], b[3], seed[3], fix[3];
  /* Off: nothing is replaced. */
  memcpy(a, rest_a, sizeof a);
  igt_override_accl(a, 5);
  assert(a[2] == rest_a[2] && !igt_sim_running() && !igt_pyro_dry());
  assert(!igt_launch(5, rest_a, rest_g, 0.0f, seed));
  enter_and_validate();
  assert(mode_now() == IGT_READY && logged("IGT CSV ok") && igt_pyro_dry());
  /* Before Launch the replay is not used. */
  memcpy(b, (float[3]){0.5f, 101000.0f, 20.0f}, sizeof b);
  igt_override_baro(b, 100);
  assert(b[0] == 0.5f);
  const uint32_t t0 = 1000;
  assert(igt_launch(t0, rest_a, rest_g, 0.4f, seed));
  assert(mode_now() == IGT_SIM && igt_sim_running() && igt_gps_simulated());
  assert(near(seed[2], 9.80665f)); /* the replay's T0 attitude row */
  assert(igt_health_bits() & IGT_HEALTH_GPS_SIM);
  /* GPS is not read before apogee: the barometer keeps it current, so the
   * first fix asked for later is current and no underrun is reported. */
  for (uint32_t t = 0; t <= 5000; t += 10) cycle(t0 + t, a, g, b, 0.3f);
  assert(igt_gps_fix(fix, t0 + 5005) && near(fix[2], 31.758f + 5005 * 1e-7f));
  assert(reason_now() == IGT_OK);
  unsigned fixes = 0;
  for (uint32_t t = 5010; t <= 20000; t += 10) {
    cycle(t0 + t, a, g, b, 0.3f);
    /* BARO: alt = t/100 m between rows 20 ms apart: exact interpolation */
    assert(near(b[0], (float)t / 100.0f));
    /* IMU: az ramps 9.80665 + t/1000 */
    assert(near(a[2], 9.80665f + (float)t / 1000.0f));
    assert(near(g[0], 1.0f));
    if (igt_gps_fix(fix, t0 + t)) {
      ++fixes;
      assert(near(fix[2], 31.758f + (float)t * 1e-7f)); /* lat */
      assert(near(fix[0], 900.0f + (float)t / 100.0f)); /* sea */
    }
  }
  assert(fixes >= 149 && fixes <= 152);
  /* The real samples are kept for the normal telemetry types. */
  assert(near(igt_real_baro()[0], 0.3f) && near(igt_real_imu()[2], 9.80665f));
  assert(near(igt_real_imu()[3], 0.1f));
  assert(igt_samples[IGT_KIND_GPS] > 0);
  igt_publish(IGT_KIND_ASCENT, (float[6]){1, 0, 0, 0, 200, 50}, 6, t0 + 20001);
  assert(igt_samples[IGT_KIND_ASCENT] == 1 && near(igt_last_sample[IGT_KIND_ASCENT][6], 200));
  assert(near(igt_last_sample[IGT_KIND_ASCENT][1], 20.001f));
  /* Exit is refused while the replay runs. */
  assert(!igt_command(IGT_CMD_EXIT, true, true) && mode_now() == IGT_SIM);
  assert(!igt_command(IGT_CMD_LIVE_PYRO, false, true));
  assert(!igt_command(IGT_CMD_ENTER, true, true));
  /* Landed: replay off, GPS back to real. */
  igt_landed();
  assert(mode_now() == IGT_DONE && !igt_sim_running() && !igt_gps_simulated());
  assert(!(igt_health_bits() & IGT_HEALTH_GPS_SIM) && logged("GPS: real"));
  assert(!igt_gps_fix(fix, t0 + 30000));
  cycle(t0 + 30010, a, g, b, 0.3f);
  assert(b[0] == 0.3f && a[2] == rest_a[2]);
  assert(reason_now() == IGT_OK);
  printf("flight ok fixes=%u\n", fixes);
}

static void scenario_gates(void)
{
  float seed[3], a[3];
  /* Entry only on the pad and before Launch. */
  assert(!igt_command(IGT_CMD_ENTER, false, false) && mode_now() == IGT_OFF);
  assert(reason_now() == IGT_E_REFUSED && logged("needs Startup"));
  assert(!igt_command(IGT_CMD_ENTER, true, true) && mode_now() == IGT_OFF);
  assert(!igt_command(IGT_CMD_LIVE_PYRO, true, false));
  assert(igt_command(IGT_CMD_ENTER, true, false));
  /* LIVE PYRO only once validated; Launch before validation: no replay. */
  assert(!igt_command(IGT_CMD_LIVE_PYRO, true, false));
  assert(!igt_launch(10, rest_a, rest_g, 0.0f, seed));
  assert(mode_now() == IGT_FAULT && reason_now() == IGT_E_REFUSED && !igt_sim_running());
  /* Re-enter, validate, arm LIVE, back to dry-run. */
  assert(igt_command(IGT_CMD_ENTER, true, false));
  igt_sd_service();
  assert(mode_now() == IGT_READY && igt_pyro_dry());
  assert(igt_command(IGT_CMD_LIVE_PYRO, true, false) && !igt_pyro_dry());
  assert(igt_health_bits() & IGT_HEALTH_LIVE);
  assert(!igt_command(IGT_CMD_LIVE_PYRO, true, true)); /* not after Launch */
  assert(igt_command(IGT_CMD_DRY_RUN, false, true) && igt_pyro_dry());
  /* Not at rest: tilted, spinning or above the pad. */
  assert(!igt_launch(10, (float[3]){0, 0, 14.0f}, rest_g, 0.0f, seed));
  assert(mode_now() == IGT_FAULT && reason_now() == IGT_E_MOVING);
  memcpy(a, rest_a, sizeof a);
  igt_override_accl(a, 20);
  assert(a[2] == rest_a[2]);
  assert(igt_command(IGT_CMD_ENTER, true, false)); igt_sd_service();
  assert(!igt_launch(10, rest_a, (float[3]){0, 30.0f, 0}, 0.0f, seed) && reason_now() == IGT_E_MOVING);
  assert(igt_command(IGT_CMD_ENTER, true, false)); igt_sd_service();
  assert(!igt_launch(10, rest_a, rest_g, 25.0f, seed) && reason_now() == IGT_E_MOVING);
  /* Stop/Abort/reset to Startup ends a running replay. */
  assert(igt_command(IGT_CMD_ENTER, true, false)); igt_sd_service();
  assert(igt_command(IGT_CMD_LIVE_PYRO, true, false));
  assert(igt_launch(1000, rest_a, rest_g, 0.0f, seed) && igt_sim_running());
  igt_stop();
  assert(mode_now() == IGT_OFF && !igt_sim_running() && !igt_pyro_dry());
  assert(!(igt_health_bits() & IGT_HEALTH_LIVE));
  memcpy(a, rest_a, sizeof a);
  igt_override_accl(a, 1500);
  assert(a[2] == rest_a[2]);
  /* Exit works when not replaying. */
  assert(igt_command(IGT_CMD_ENTER, true, false)); igt_sd_service();
  assert(igt_command(IGT_CMD_EXIT, true, false) && mode_now() == IGT_OFF);
  printf("gates ok\n");
}

static void scenario_real_motion(void)
{
  float a[3], g[3], b[3], seed[3];
  enter_and_validate();
  assert(igt_launch(0, rest_a, rest_g, 0.0f, seed) && igt_pyro_dry());
  for (uint32_t t = 10; t <= 90; t += 10) cycle(t, a, g, b, 60.0f);
  assert(igt_sim_running());
  cycle(100, a, g, b, 60.0f);
  assert(mode_now() == IGT_FAULT && reason_now() == IGT_E_REAL_MOTION);
  assert(!igt_sim_running() && !igt_pyro_dry() && logged("real motion"));
  cycle(110, a, g, b, 61.0f);
  assert(b[0] == 61.0f); /* real sensors */
  printf("real motion ok\n");
}

static void scenario_data_end(void)
{
  float a[3], g[3], b[3], seed[3];
  enter_and_validate();
  assert(igt_launch(0, rest_a, rest_g, 0.0f, seed));
  uint32_t t;
  for (t = 10; t <= 40000 && igt_sim_running(); t += 10) {
    cycle(t, a, g, b, 0.0f);
    if (t > 6000 && igt_sim_running()) assert(near(b[0], 60.0f)); /* holds the last row */
  }
  assert(mode_now() == IGT_DONE && reason_now() == IGT_I_DATA_ENDED);
  assert(t > 36000 && t < 36100);
  assert(logged("data ended"));
  printf("data end ok t=%u\n", (unsigned)t);
}

static void scenario_sd_fail(void)
{
  igt_fail_sd = 1;
  enter_and_validate();
  printf("mode=%u reason=%u\n", mode_now(), reason_now());
}

int main(int argc, char **argv)
{
  snprintf(igt_root, sizeof igt_root, "%s", argv[2]);
  if (!strcmp(argv[1], "validate")) scenario_validate();
  else if (!strcmp(argv[1], "numbers")) scenario_numbers();
  else if (!strcmp(argv[1], "flight")) scenario_flight();
  else if (!strcmp(argv[1], "gates")) scenario_gates();
  else if (!strcmp(argv[1], "real_motion")) scenario_real_motion();
  else if (!strcmp(argv[1], "data_end")) scenario_data_end();
  else if (!strcmp(argv[1], "sd_fail")) scenario_sd_fail();
  else return 2;
  return 0;
}
'''

IMU_HEADER = 't_ms,ax_mps2,ay_mps2,az_mps2,gx_dps,gy_dps,gz_dps'
BARO_HEADER = 't_ms,alt_m,pressure_pa,temp_c'
GPS_HEADER = 't_ms,lat_deg,lon_deg,alt_msl_m'


def imu_rows(end=40000, step=10):
    return [f'{t},0,0,{9.80665 + t / 1000:.5f},1,0,0' for t in range(0, end + 1, step)]


def baro_rows(end=40000, step=20):
    return [f'{t},{t / 100:.2f},101000,20' for t in range(0, end + 1, step)]


def gps_rows(end=40000, step=100):
    return [f'{t},{31.758 + t * 1e-7:.7f},-102.046,{900 + t / 100:.2f}' for t in range(0, end + 1, step)]


class GroundTestReplay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        work = Path(cls.tmp.name)
        (work / 'ground_test_host_port.h').write_text(PORT)
        (work / 'driver.c').write_text(DRIVER)
        cls.exe = work / 'igt'
        subprocess.run(CC + ['-DIGT_HOST_TEST', '-DFC_GROUND_TEST=1',
                             '-I', str(work), '-I', str(ROOT / 'Core/Inc'),
                             '-I', str(ROOT / 'Core/Src'),
                             str(work / 'driver.c'), '-lm', '-o', str(cls.exe)],
                       check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def card(self, files):
        root = tempfile.mkdtemp(dir=self.tmp.name)
        (Path(root) / 'IGT').mkdir()
        for name, text in files.items():
            (Path(root) / 'IGT' / name).write_bytes(text.encode())
        return root

    def run_driver(self, scenario, files):
        result = subprocess.run([str(self.exe), scenario, self.card(files)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def nominal(self, **override):
        files = {
            'IMU0.CSV': '\n'.join([IMU_HEADER] + imu_rows()) + '\n',
            # Two baro segments, CRLF, comments and blank lines.
            'BARO0.CSV': '\r\n'.join(['# replayed barometer', BARO_HEADER] + baro_rows(20000)) + '\r\n',
            'BARO1.CSV': '\n'.join([BARO_HEADER, ''] + baro_rows(40000)[1001:]),
            'GPS0.CSV': '\n'.join([GPS_HEADER] + gps_rows()) + '\n',
        }
        files.update(override)
        return {k: v for k, v in files.items() if v is not None}

    def validate(self, files):
        out = self.run_driver('validate', files)
        m = re.search(r'mode=(\d+) reason=(\d+) gps=(\d+)', out)
        return int(m.group(1)), int(m.group(2)), int(m.group(3)), out

    def test_numbers_and_rows(self):
        self.assertIn('numbers ok', self.run_driver('numbers', self.nominal()))

    def test_valid_set_reaches_ready_with_primed_rings(self):
        mode, reason, gps, out = self.validate(self.nominal())
        self.assertEqual((mode, reason, gps), (2, 0, 1), out)
        self.assertIn('imu=32 baro=32', out)
        self.assertIn('IGT CSV ok: IMU 4001 rows 40s, BARO 2001 rows 40s, GPS 401 rows', out)

    def test_gps_is_optional(self):
        mode, reason, gps, out = self.validate(self.nominal(**{'GPS0.CSV': None}))
        self.assertEqual((mode, reason, gps), (2, 0, 0), out)

    def test_validation_failures_are_reported_with_file_and_line(self):
        bad_range = '\n'.join([IMU_HEADER] + imu_rows()[:5] + ['50,0,0,999,0,0,0'])
        cases = [
            ({'IMU0.CSV': None}, 2, 'IMU0.CSV:0 file missing'),
            ({'BARO0.CSV': 'time,alt\n0,1\n'}, 3, 'BARO0.CSV:1 bad header'),
            ({'BARO0.CSV': ''}, 3, 'BARO0.CSV:0 bad header'),
            ({'BARO0.CSV': f'{BARO_HEADER}\n0,1,101000,20\n20,abc,101000,20\n'}, 4, 'BARO0.CSV:3 parse error'),
            ({'BARO0.CSV': f'{BARO_HEADER}\n0,1,101000,20\n0,2,101000,20\n'}, 5, 'BARO0.CSV:3 time not increasing'),
            # Time must also increase across segments.
            ({'BARO1.CSV': f'{BARO_HEADER}\n19000,1,101000,20\n'}, 5, 'BARO1.CSV:2 time not increasing'),
            ({'IMU0.CSV': bad_range}, 6, 'IMU0.CSV:7 value out of range'),
            ({'GPS0.CSV': f'{GPS_HEADER}\n0,10.0,-102.046,900\n'}, 6, 'GPS0.CSV:2 value out of range'),
            ({'IMU0.CSV': '\n'.join([IMU_HEADER] + imu_rows(4000))}, 7, 'too short'),
            ({'GPS0.CSV': f'{GPS_HEADER}\n0,31.758,-102.046,900\n'}, 7, 'too short'),
            ({'BARO0.CSV': f'{BARO_HEADER}\n0,1,101000,20' + ' ' * 130 + '\n'}, 4, 'BARO0.CSV:2 parse error'),
            ({'BARO0.CSV': BARO_HEADER + '\n' + 'x' * 300 + '\n'}, 4, 'parse error'),
        ]
        for override, code, text in cases:
            with self.subTest(text=text):
                mode, reason, _, out = self.validate(self.nominal(**override))
                self.assertEqual((mode, reason), (5, code), out)
                self.assertIn(f'IGT {text}' if ':' in text else text, out)

    def test_sd_unavailable_faults(self):
        out = self.run_driver('sd_fail', self.nominal())
        self.assertIn('mode=5 reason=1', out)

    def test_replay_interpolates_and_switches_gps_to_real_at_landing(self):
        self.assertIn('flight ok', self.run_driver('flight', self.nominal()))

    def test_mode_gates_reset_and_live_pyro(self):
        self.assertIn('gates ok', self.run_driver('gates', self.nominal()))

    def test_real_motion_abandons_replay_and_clears_dry_run(self):
        self.assertIn('real motion ok', self.run_driver('real_motion', self.nominal()))

    def test_data_end_holds_then_ends_replay(self):
        short = {
            'IMU0.CSV': '\n'.join([IMU_HEADER] + [f'{t},0,0,9.80665,0,0,0' for t in range(0, 6001, 10)]),
            'BARO0.CSV': '\n'.join([BARO_HEADER] + [f'{t},{t / 100:.2f},101000,20' for t in range(0, 6001, 20)]),
            'GPS0.CSV': None,
            'BARO1.CSV': None,
        }
        self.assertIn('data end ok', self.run_driver('data_end', self.nominal(**short)))


PULSE_STUB = r'''
#include <assert.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "fc_sd_events.h"
#define FC_GROUND_TEST 1
#define CO2_ASSERT_INTERVAL 500
#define REEF_ASSERT_INTERVAL 500
#define Parachute_Deployed 1
#define Parachute_Expanded 2
#define CO2_Asserted 4
#define REEF_Asserted 8
#define option(x) (x)
#define TX_TIMER_TICKS_PER_SECOND 1000
atomic_uint_fast32_t g_conf;
static unsigned mask, co2, reef, records, stops, dry_notes;
static bool dry = true;
static struct { uint32_t ODR; } gpio;
#define PYRO_PORT (&gpio)
#define CO2_PIN 1U
#define REEF_PIN 2U
static bool fc_watchdog_can_actuate(void) { return true; }
static bool fc_recovery_inhibits_pyro(void) { return false; }
static bool fc_recovery_inhibits_manual_pyro(void) { return false; }
static bool fc_watchdog_record_deployment(bool r, bool m) { (void)r; (void)m; ++records; return true; }
static uint32_t __get_PRIMASK(void) { return mask; }
static void __disable_irq(void) { mask = 1; }
static void __set_PRIMASK(uint32_t v) { mask = v; }
static void fc_recovery_ota_reset_retire_locked(void) {}
static void co2_high(void) { co2 = 1; }
static void reef_high(void) { reef = 1; }
static void co2_low(void) { co2 = 0; }
static void reef_low(void) { reef = 0; }
static void HAL_SuspendTick(void) {}
static unsigned HAL_GetTick(void) { return 0; }
static void HAL_IncTick(void) {}
static bool igt_pyro_dry(void) { assert(mask); return dry; }
static void igt_note_dry(bool r) { assert(mask); dry_notes |= r ? 2U : 1U; }
static void igt_stop(void) { assert(mask); ++stops; dry = false; }
'''

PULSE_MAIN = r'''
int main(void)
{
  /* Dry-run: no GPIO, no watchdog journal entry, no asserted bit, but the
   * state machine sees the deployment and the SD log records it. */
  deployment_pulse_start(false, false);
  deployment_pulse_start(true, true); /* the operator's manual Expand too */
  assert(!co2 && !reef && records == 0 && !mask);
  assert((g_conf & (Parachute_Deployed | Parachute_Expanded)) == (Parachute_Deployed | Parachute_Expanded));
  assert(!(g_conf & (CO2_Asserted | REEF_Asserted)));
  assert(dry_notes == 3);
  fc_sd_event e; unsigned n = 0;
  while (fc_sd_peek_event(&e)) { assert(e.kind == FC_LOG_PYRO_DRY_RUN); fc_sd_consume_event(); ++n; }
  assert(n == 2);
  /* LIVE PYRO: the real outputs fire. */
  dry = false;
  deployment_pulse_start(false, false);
  assert(co2 && records == 1 && (g_conf & CO2_Asserted));
  /* Abort/Stop ends the ground test with the pulse. */
  dry = true;
  mask = 1; deployment_pulse_cancel_locked(); mask = 0;
  assert(!co2 && stops == 1 && !dry);
  return 0;
}
'''


class GroundTestPyroGate(unittest.TestCase):
    def test_dry_run_versus_live_and_stop(self):
        source = (ROOT / 'Core/Src/deployment_pulse.c').read_text()
        source = '\n'.join(x for x in source.splitlines() if not x.startswith('#include'))
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'pulse.c'
            c.write_text(PULSE_STUB + source + PULSE_MAIN)
            exe = Path(tmp) / 'pulse'
            subprocess.run(CC + ['-I', str(ROOT / 'Core/Inc'), str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


def extract(text, signature):
    start = text.index(signature)
    end = text.index('{', start) + 1
    depth = 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


GPS_STUB = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#define FC_GROUND_TEST 1
#define GPS_AVAILABLE 1
typedef struct { float x, y, z; } f_xyz;
typedef struct { float sea, lon, lat; } kf_gps;
typedef struct { int lock; } spinlock;
static struct { kf_gps rail; f_xyz coords_buf; spinlock rflock; bool updated; } rfboard;
static bool simulated = true;
static unsigned heartbeats, now = 1000;
enum { HeartbeatRF };
static void fc_lock(spinlock *l) { (void)l; }
static void fc_unlock(spinlock *l) { (void)l; }
static void timer_update(int t) { (void)t; ++heartbeats; }
static uint32_t now_ms(void) { return now; }
static bool igt_gps_simulated(void) { return simulated; }
static bool igt_gps_fix(float *g, uint32_t t) { (void)t; g[0] = 950; g[1] = -102.05f; g[2] = 31.76f; return true; }
'''

GPS_MAIN = r'''
int main(void)
{
  kf_gps out;
  rfboard.rail = (kf_gps){890, -102.0f, 31.7f};       /* real pad reference */
  rfboard.coords_buf = (f_xyz){31.71f, -102.01f, 891}; rfboard.updated = true;
  /* Replay: replayed fix, rail follows the replay's pad, real fix kept. */
  assert(fetch_gps_data(&out) && out.lat == 31.76f && out.sea == 950);
  assert(rfboard.rail.lat == 31.76f && heartbeats == 1 && rfboard.updated);
  assert(fetch_gps_data(&out) && rfboard.rail.lat == 31.76f);
  /* Landed: real GPS again, real rail restored, the waiting real fix used. */
  simulated = false;
  assert(fetch_gps_data(&out) && out.lat == 31.71f && out.sea == 891);
  assert(rfboard.rail.lat == 31.7f && rfboard.rail.sea == 890 && !rfboard.updated);
  assert(!fetch_gps_data(&out));
  return 0;
}
'''


class GroundTestGpsSwitch(unittest.TestCase):
    def test_replayed_gps_until_landing_then_real(self):
        text = (ROOT / 'Core/Src/distribution.c').read_text()
        real = extract(text, 'static inline bool fetch_real_gps(kf_gps *buf)')
        wrapper = extract(text, 'static inline bool fetch_gps_data(kf_gps *buf)')
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'gps.c'
            c.write_text(GPS_STUB + real + '\n' + wrapper + GPS_MAIN)
            exe = Path(tmp) / 'gps'
            subprocess.run(CC + [str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


GUARDED_CALLS = re.compile(
    r'\bigt_(override_\w+|launch|gps_fix|gps_simulated|publish|command|real_\w+|'
    r'sd_service|landed|stop|engaged|pyro_dry|note_dry|health_bits)\s*\(|\bkalman_seed_attitude\s*\(')


def unguarded_calls(text):
    """Lines calling an IGT hook outside an '#if FC_GROUND_TEST' block."""
    depth_stack, bad = [], []
    for number, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        if stripped.startswith('#if'):
            depth_stack.append('FC_GROUND_TEST' in stripped and '!' not in stripped)
        elif stripped.startswith('#else') and depth_stack:
            depth_stack[-1] = False if depth_stack[-1] else depth_stack[-1]
        elif stripped.startswith('#endif') and depth_stack:
            depth_stack.pop()
        elif GUARDED_CALLS.search(line) and not stripped.startswith(('/*', '*', '//')):
            if not any(depth_stack):
                bad.append(f'{number}: {stripped}')
    return bad


class GroundTestUnreachableInFlightImages(unittest.TestCase):
    def test_every_hook_is_compiled_out_without_the_option(self):
        for path in sorted((ROOT / 'Core').rglob('*.[ch]')):
            if path.name in ('ground_test.c', 'ground_test.h'):
                continue
            with self.subTest(path=path.name):
                self.assertEqual(unguarded_calls(path.read_text()), [])

    def test_option_is_off_by_default_and_explicit_in_build_py(self):
        cmake = (ROOT / 'CMakeLists.txt').read_text()
        self.assertIn('option(INTEGRATED_GROUND_TEST "Ground-test CSV replay image" OFF)', cmake)
        self.assertIn('FC_GROUND_TEST=1', cmake)
        build = (ROOT / 'build.py').read_text()
        self.assertIn('"-DINTEGRATED_GROUND_TEST=" + ("ON" if options.get("groundtest") else "OFF")', build)
        source = (ROOT / 'Core/Src/ground_test.c').read_text()
        self.assertTrue(source.split('#include "ground_test.h"', 1)[1].lstrip().startswith('#if FC_GROUND_TEST'))

    def test_flight_header_stubs_are_inert(self):
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'flight.c'
            c.write_text('#include <assert.h>\n#include "ground_test.h"\n'
                         '#if FC_GROUND_TEST\n#error flight image\n#endif\n'
                         'int main(void) { assert(!igt_sim_running() && !igt_gps_simulated() &&'
                         ' !igt_pyro_dry() && !igt_engaged() && igt_health_bits() == 0);'
                         ' igt_stop(); igt_landed(); igt_sd_service(); igt_note_dry(true);'
                         ' assert(!igt_pyro_dry()); return 0; }\n')
            exe = Path(tmp) / 'flight'
            subprocess.run(CC + ['-I', str(ROOT / 'Core/Inc'), str(c), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
        # The replay entry points do not even exist for a flight image.
        header = (ROOT / 'Core/Inc/ground_test.h').read_text()
        flight_part = header.split('#else', 1)[1]
        for name in ('igt_launch', 'igt_override_', 'igt_gps_fix', 'igt_publish', 'igt_command'):
            self.assertNotIn(name, flight_part)

    def test_reset_exits_the_mode(self):
        # RAM only: no ground-test state is part of the watchdog resume image
        # or any persistent store, and no checkpoint is taken while engaged.
        for path in sorted((ROOT / 'Core/Src').glob('*.c')):
            for line in path.read_text().splitlines():
                if 'FC_RESUME_REGION(' in line:
                    self.assertNotRegex(line, r'igt|ground|streams|ring_')
        source = (ROOT / 'Core/Src/ground_test.c').read_text()
        self.assertNotIn('persistent_store', source)
        self.assertNotIn('FC_RESUME_REGION', source)
        checkpoint = extract((ROOT / 'Core/Src/fc_watchdog_recovery.c').read_text(),
                             'void fc_watchdog_checkpoint(void)')
        self.assertRegex(checkpoint, r'#if FC_GROUND_TEST\s+if \(igt_engaged\(\)\) return;\s+#endif')
        startup = extract((ROOT / 'Core/Src/recovery.c').read_text(),
                          'bool fc_recovery_control_finalize_startup(void)')
        self.assertIn('igt_stop();', startup)

    def test_replay_starts_only_from_launch_acceptance(self):
        text = (ROOT / 'Core/Src/recovery.c').read_text()
        launch = extract(text, 'static inline void enter_launch(bool noconfirm)')
        # After admission and the Launch-accepted event, before the evaluator.
        self.assertLess(launch.index('fc_recovery_admit_new_flight'), launch.index('igt_launch('))
        self.assertLess(launch.index('igt_launch('), launch.index('tx_thread_resume(&evaluation_task)'))
        self.assertEqual(text.count('igt_launch('), 1)
        action = extract(text, 'static inline void process_action(fc_msg cmd, bool internal)')
        self.assertIn('if (!internal)', action[action.index('case IGT_Enter:'):])


if __name__ == '__main__':
    unittest.main()
