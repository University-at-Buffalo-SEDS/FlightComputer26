from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BarometerPublicationGateTests(unittest.TestCase):
    def test_central_rate_gate_controls_fresh_baro_without_a_second_phase(self):
        source = (ROOT / "Core/Src/distribution.c").read_text()
        body = source.split("static inline bool maybe_log_measm", 1)[1].split(
            "/* Distribution for Ascent */", 1)[0]
        preamble = r'''
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "telemetry_rate.h"
#include "sedsnet_config.h"
typedef uint32_t fu32;
typedef enum { IMU, Baro, MEMS_Devices } devid;
enum { IMULocal, IMURemote, BaroLocal, BaroRemote };
typedef struct { unsigned tim_sd, tim_gnd, size; uint32_t kind_sd, kind_gnd; } log_lookup;
static struct { uint32_t sd, gnd; } rates;
static uint32_t now;
static unsigned timer_calls, baro_publications, imu_publications;
static bool producer_timer_ready;
static bool timer_probe_ground(unsigned timer, uint32_t interval) {
    (void)timer; (void)interval; ++timer_calls; return producer_timer_ready;
}
static void log_f32(uint32_t type, unsigned count, const void *data) {
    (void)count; (void)data;
    if (fc_telemetry_rate_allow(type, now)) {
        if (type == SEDS_DT_BAROMETER_DATA) ++baro_publications;
        if (type == SEDS_DT_IMU_DATA) ++imu_publications;
    }
}
#define log_periodic_f32 log_f32
'''
        checks = r'''
int main(void) {
    float sample[6] = {0};
    rates.gnd = fc_telemetry_period_ms();
    producer_timer_ready = false;
    now = 200;
#ifdef TELEMETRY_ENABLED
    maybe_log_measm(Baro, sample);
    assert(baro_publications == 1 && timer_calls == 0);
    now = 250;
    maybe_log_measm(Baro, sample);
    assert(baro_publications == 1); /* Actual central cap still refuses early samples. */
    now = 400;
    maybe_log_measm(Baro, sample);
    assert(baro_publications == 2);
    maybe_log_measm(IMU, sample);
    assert(imu_publications == 0 && timer_calls == 1); /* No extra high-rate IMU lock traffic. */
    rates.gnd = 15000;
    now = 600;
    maybe_log_measm(Baro, sample);
    assert(baro_publications == 2 && timer_calls == 2);
    producer_timer_ready = true;
    now = 15000;
    maybe_log_measm(Baro, sample);
    assert(baro_publications == 3 && timer_calls == 3);
    producer_timer_ready = false;
    rates.gnd = UINT32_MAX;
    now = 16000;
    maybe_log_measm(Baro, sample);
    assert(baro_publications == 3 && timer_calls == 4);
#else
    maybe_log_measm(Baro, sample);
    assert(baro_publications == 0 && timer_calls == 1);
    producer_timer_ready = true;
    maybe_log_measm(Baro, sample);
    assert(baro_publications == 1 && timer_calls == 2);
#endif
}
'''
        with tempfile.TemporaryDirectory() as directory:
            temporary = Path(directory)
            (temporary / "sedsnet.h").write_text(
                "#include <stdint.h>\ntypedef uint32_t SedsDataType;\ntypedef uint32_t SedsDataEndpoint;\n")
            executable = temporary / "barometer-gate"
            for enabled in (False, True):
                command = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                           "-fsanitize=address,undefined", "-I", str(temporary),
                           "-I", str(ROOT / "Core/Inc")]
                if enabled:
                    command.append("-DTELEMETRY_ENABLED=1")
                command += [str(ROOT / "Core/Src/telemetry_rate.c"), "-x", "c", "-", "-o", str(executable)]
                subprocess.run(command,
                               input=preamble + "static inline bool maybe_log_measm" + body + checks,
                               text=True, check=True)
                subprocess.run([str(executable)], check=True, timeout=5)
