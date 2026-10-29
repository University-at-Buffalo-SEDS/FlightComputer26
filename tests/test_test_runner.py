import contextlib
import io
import unittest
from unittest import mock
from pathlib import Path

import build


class TestRunnerReportingTests(unittest.TestCase):
    def test_success_summary_is_explicit(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            build.print_test_summary([
                ("Python unit tests", "PASS"),
                ("GoogleTest unit tests", "PASS"),
            ])
        self.assertIn("All 2 test stages passed", output.getvalue())

    def test_failed_stage_has_actionable_help(self):
        results = []

        def fail():
            raise RuntimeError("compiler rejected test source")

        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            with self.assertRaises(RuntimeError) as raised:
                build.run_test_stage(results, "GoogleTest unit tests", fail)
        self.assertEqual(results, [("GoogleTest unit tests", "FAIL")])
        self.assertIn("Possible solutions", str(raised.exception))
        self.assertIn("C++17 compiler", str(raised.exception))

    def test_ctest_zero_test_guard_is_enabled(self):
        source = Path(build.__file__).read_text(encoding="utf-8")
        self.assertIn("--no-tests=error", source)

    def test_panic_match_is_production_code(self):
        root = Path(build.__file__).resolve().parent
        cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
        hooks = (root / "Core" / "Src" / "fchooks.c").read_text(encoding="utf-8")
        self.assertIn("Core/Src/panic_match.c", cmake)
        self.assertIn("fc_panic_message_contains", hooks)




class DaqObserverCadenceTests(unittest.TestCase):
    def test_current_and_legacy_observer_environment_matches_log_assertion(self):
        from sim import run_full
        with mock.patch.dict(run_full.os.environ, {}, clear=True):
            current = run_full.daq_loadcell_observer_config()
        self.assertEqual(current["environment"], {"GS_SIM_DAQ_LOADCELL_HZ": "40"})
        self.assertIn("40 Hz", current["assertion"]["name"])
        self.assertEqual(current["assertion"]["contains"],
                         "full-bay DAQ raw loadcell 40 Hz stream reached GroundStation")
        for rate in (1, 250, 1000):
            with self.subTest(rate=rate), mock.patch.dict(
                run_full.os.environ, {"SEDS_FIRMWARE_SIM_DAQ_LOADCELL_HZ": str(rate)}, clear=True
            ):
                legacy = run_full.daq_loadcell_observer_config()
                self.assertEqual(legacy["environment"], {"GS_SIM_DAQ_LOADCELL_HZ": str(rate)})
                self.assertIn(f"{rate} Hz", legacy["assertion"]["name"])
                self.assertIn(f"{rate} Hz", legacy["assertion"]["contains"])
                self.assertEqual(legacy["assertion"]["node"], "groundstation")

    def test_invalid_rate_fails_before_docker_readiness_or_network_work(self):
        from sim import run_full
        for value in ("", "0", "-1", "1001", "40.0", "nan", "inf", "bad"):
            with self.subTest(value=value), mock.patch.dict(
                run_full.os.environ, {"SEDS_FIRMWARE_SIM_DAQ_LOADCELL_HZ": value}, clear=True
            ), mock.patch.object(run_full.shutil, "which") as which, mock.patch.object(
                run_full.subprocess, "run"
            ) as execute:
                with self.assertRaisesRegex(RuntimeError, "SEDS_FIRMWARE_SIM_DAQ_LOADCELL_HZ"):
                    run_full.require_docker()
                which.assert_not_called()
                execute.assert_not_called()
                with mock.patch.object(run_full, "require_docker") as ready:
                    with self.assertRaisesRegex(RuntimeError, "SEDS_FIRMWARE_SIM_DAQ_LOADCELL_HZ"):
                        run_full.run_network_simulation(mock.Mock(), Path("/board"), "stm32g4")
                    ready.assert_not_called()


if __name__ == "__main__":
    unittest.main()
