import contextlib
import io
import unittest
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


if __name__ == "__main__":
    unittest.main()
