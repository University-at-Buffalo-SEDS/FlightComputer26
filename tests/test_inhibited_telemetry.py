"""Run the production distribution entry with the actuation interlock closed."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class InhibitedTelemetry(unittest.TestCase):
    def test_inhibited_boot_streams_without_entering_flight_sequence(self):
        source = (ROOT / "Core/Src/distribution.c").read_text()
        entry = source[source.index("void distribution_entry(ULONG _)"):
                       source.index("UINT create_distribution_task")]
        harness = r'''#include <assert.h>
typedef unsigned ULONG;
typedef unsigned char fu8;
typedef unsigned fu32;
static unsigned g_conf, g_fc_watchdog_resumed;
static int allowed, fills, streams, estimator;
enum { BOARD_WATCHDOG_ACQUISITION=1, BOARD_WATCHDOG_SAFETY=2,
       TX_TIMER_TICKS_PER_SECOND=100, Eval_Abort_Flag=1, Using_Ascent_KF=2,
       Ascent_KF_Staged=4, Acq=0 };
#define option(x) (x)
#define load(p, order) (*(p))
#define MrAnalog(x) while (x)
#define WE_ARE_SO_BACK 0
static int fc_watchdog_can_actuate(void) { return allowed; }
static void fill_sequence_states(void) { assert(allowed); fills++; }
static void data_streaming_mode(void) { assert(!allowed); streams++; allowed=1; }
static void board_watchdog_progress(unsigned mask) { (void)mask; }
static void tx_thread_sleep(unsigned ticks) { (void)ticks; }
static void for_ascent_update(unsigned conf) { (void)conf; estimator++; }
static void for_ascent_predict(unsigned conf, unsigned char *imu)
{ (void)conf; (void)imu; estimator++; }
static void descent_full_cycle(unsigned conf) { (void)conf; estimator++; }
''' + entry + r'''
int main(void) {
  allowed=0; distribution_entry(0);
  assert(streams==1 && fills==0 && estimator==0);
  allowed=1; streams=0; distribution_entry(0);
  assert(fills==1 && streams==0 && estimator==0);
  g_fc_watchdog_resumed=1; distribution_entry(0);
  assert(fills==1 && streams==0 && estimator==0);
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "test.c").write_text(harness)
            subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-Wall",
                            "-Wextra", "-Werror", "-Wno-unused-parameter",
                            str(path / "test.c"), "-o", str(path / "test")], check=True)
            subprocess.run([str(path / "test")], check=True)

    def test_recovery_initializes_sensors_before_command_loop_on_cold_boot(self):
        source = (ROOT / "Core/Src/recovery.c").read_text()
        startup = source[source.index("void recovery_entry"):
                         source.index("tx_timer_activate(&monotonic_checks)")]
        block = startup[startup.index("if (g_fc_watchdog_resumed)\n"):]
        self.assertIn("sensor_init_supervised(Wild_Mask);\n  if (g_fc_watchdog_resumed)", block)
        distribution = (ROOT / "Core/Src/distribution.c").read_text()
        streaming = distribution[distribution.index("static inline void data_streaming_mode"):
                                 distribution.index("/* Stage 1")]
        self.assertNotIn("request_ignition", streaming)
        self.assertNotIn("deployment", streaming)
