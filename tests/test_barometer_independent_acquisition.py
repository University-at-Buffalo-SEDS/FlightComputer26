"""Execute the production ascent distributor with an absent IMU."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BarometerIndependentAcquisition(unittest.TestCase):
    def test_barometer_keeps_publishing_and_stages_updates_without_imu(self):
        source = (ROOT / 'Core/Src/distribution.c').read_text()
        start = source.index('static inline void for_ascent_predict(')
        end = source.index('/* Distribution for Descent', start)
        production = source[start:end]
        harness = r'''
#include <assert.h>
#include <stdbool.h>
typedef unsigned fu32;
typedef unsigned char fu8;
typedef struct { float x,y,z; } f_xyz;
typedef struct { float pressure, temperature, altitude; } baro;
enum { Baro=0, IMU=1, Gyro_Mask=1, Accl_Mask=2, Baro_Mask=4,
       TX_OR=0, Eval_Focused=8 };
static struct { baro baro; f_xyz accl,gyro; } meas;
static int meas_locks[2], eval_stage;
static unsigned published, staged;
#define option(value) (value)
#define sweetbench_start(value) ((void)(value))
#define fc_lock(value) ((void)(value))
#define fc_concede(value) ((void)(value))
static bool try_fetch_baro(baro *sample) {
  sample->pressure = 100000.0f + published;
  return true;
}
static bool validate_baro(baro *sample, fu32 conf) {
  (void)sample; (void)conf; return true;
}
static bool try_fetch_gyro(f_xyz *sample) { (void)sample; return false; }
static bool try_fetch_accl(f_xyz *sample) { (void)sample; return false; }
static bool validate_gyro(f_xyz *sample, fu32 conf) {
  (void)sample; (void)conf; return true;
}
static bool validate_accl(f_xyz *sample, fu32 conf) {
  (void)sample; (void)conf; return true;
}
static void maybe_log_measm(unsigned device, const void *sample) {
  (void)sample; assert(device == Baro); ++published;
}
static void tx_event_flags_set(int *group, unsigned bits, unsigned operation) {
  (void)group; (void)operation; staged |= bits;
}
static void tx_thread_relinquish(void) {}
'''
        harness += production + r'''
int main(void) {
  fu8 imu = 0;
  for(unsigned step=0; step<100; ++step) {
    staged=0;
    for_ascent_predict(0, &imu);
    assert(published == step+1);
    assert(meas.baro.pressure == 100000.0f + step);
    assert(staged == Baro_Mask);
    assert(imu == 0);
  }
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'test.c').write_text(harness)
            subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall',
                            '-Wextra', '-Werror', '-Wno-unused-function',
                            str(path / 'test.c'), '-o',
                            str(path / 'test')], check=True)
            subprocess.run([str(path / 'test')], check=True)
