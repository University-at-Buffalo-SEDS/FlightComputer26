import os
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(os.environ.get("FC_RATE_TEST_ROOT", pathlib.Path(__file__).resolve().parents[1]))
STREAMS = ["GPS_DATA", "GYRO_DATA", "ACCEL_DATA", "BATTERY_VOLTAGE", "BATTERY_CURRENT", "BAROMETER_DATA", "FUEL_FLOW", "FUEL_TANK_PRESSURE", "KG1000", "KG50", "GPS_SATELLITE_NUMBER", "EULER_ANGLES", "IMU_DATA"]


class TelemetryRateTests(unittest.TestCase):
    def compile_rate(self, rate, expected, valid=True):
        with tempfile.TemporaryDirectory() as directory:
            tmp = pathlib.Path(directory)
            # Use the board's real schema constants. Only the external ABI's
            # integer typedefs are needed to compile this isolated C function.
            (tmp / "sedsnet.h").write_text("#include <stdint.h>\ntypedef uint32_t SedsDataType;\ntypedef uint32_t SedsDataEndpoint;\n")
            code = '#include "telemetry_rate.h"\n#include "fcconfig.h"\n#include "sedsnet_config.h"\n#include <assert.h>\nint main(void) {\n'
            code += f"assert(LOG_RATE_GND == {expected});\nassert(fc_telemetry_period_ms() == {expected});\n"
            code += "const uint32_t types[] = {" + ",".join("SEDS_DT_" + name for name in STREAMS) + "};\n"
            code += '''
  for (unsigned i = 0; i < sizeof(types)/sizeof(types[0]); ++i) {
    assert(fc_telemetry_rate_allow(types[i], 0));
    assert(!fc_telemetry_rate_allow(types[i], EXPECTED / 2));
    assert(!fc_telemetry_rate_allow(types[i], EXPECTED - 1));
    assert(fc_telemetry_rate_allow(types[i], EXPECTED));
    assert(fc_telemetry_rate_allow(types[i], UINT32_MAX - 10));
    assert(!fc_telemetry_rate_allow(types[i], (uint32_t)(UINT32_MAX - 10U + EXPECTED - 1U)));
    assert(fc_telemetry_rate_allow(types[i], (uint32_t)(UINT32_MAX - 10U + EXPECTED)));
    unsigned count = 0;
    for (uint32_t now = 2 * EXPECTED; now < 12 * EXPECTED; ++now)
      count += fc_telemetry_rate_allow(types[i], now);
    assert(count == 10);
  }
  // These current IDs previously collided with unrelated v3 sensor IDs.
  const uint32_t controls[] = { SEDS_DT_FLIGHT_COMMAND, SEDS_DT_MESSAGE_DATA,
    SEDS_DT_WARNING, SEDS_DT_ABORT, SEDS_DT_FLIGHT_STATE };
  for (unsigned i = 0; i < sizeof(controls)/sizeof(controls[0]); ++i) {
    assert(fc_telemetry_rate_allow(controls[i], 0));
    assert(fc_telemetry_rate_allow(controls[i], 0));
  }
'''.replace("EXPECTED", str(expected)) + "}\n"
            cmd = ["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", "-I", str(tmp), "-I", str(ROOT / "Core/Inc")]
            if rate is not None:
                cmd += [f"-DFC_TELEMETRY_RATE_HZ={rate}"]
            cmd += [str(ROOT / "Core/Src/telemetry_rate.c"), "-x", "c", "-", "-o", str(tmp / "rate-test")]
            result = subprocess.run(cmd, input=code, text=True, capture_output=True)
            if valid:
                self.assertEqual(result.returncode, 0, result.stderr)
                subprocess.run([str(tmp / "rate-test")], check=True)
            else:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("must be a whole number", result.stderr)

    def test_default_five_hz(self):
        self.compile_rate(None, 200)

    def test_twenty_hz_updates_producer_and_limiter(self):
        self.compile_rate(20, 50)

    def test_board_override_four_hz(self):
        self.compile_rate(4, 250)

    def test_one_khz_boundary(self):
        self.compile_rate(1000, 1)

    def test_invalid_rates_rejected(self):
        for rate in (0, 1001):
            with self.subTest(rate=rate):
                self.compile_rate(rate, 1000, valid=False)

if __name__ == "__main__":
    unittest.main()
