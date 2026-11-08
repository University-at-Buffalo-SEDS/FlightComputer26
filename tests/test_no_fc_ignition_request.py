"""The FC must not send its own ignition request on Launch.

Ignition is owned by the Valve and Actuator boards. The old FC request sent
valve command 14, which the Valve Board ignores and the Actuator Board never
receives, while spending distribution-task stack on the Launch path.
"""
import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class NoFcIgnitionRequestTests(unittest.TestCase):
    def test_launch_path_sends_no_valve_command(self):
        for rel in ("Core/Src/distribution.c", "Core/Inc/fcapi.h"):
            text = (ROOT / rel).read_text()
            self.assertNotIn("request_ignition", text, rel)
            self.assertNotIn("log_valve_board_command(", text, rel)

    def test_ignition_command_constant_removed(self):
        for rel in ("Core/Inc/fcconfig.h", "Core/Test/simulation.h"):
            self.assertNotIn("IGNITION_COMMAND", (ROOT / rel).read_text(), rel)

    def test_launch_still_enters_flight_mode(self):
        text = (ROOT / "Core/Src/distribution.c").read_text()
        self.assertIn('"ignition; flight mode"', text)


if __name__ == "__main__":
    unittest.main()
