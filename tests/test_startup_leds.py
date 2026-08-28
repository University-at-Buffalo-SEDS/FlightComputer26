from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class StartupLedTests(unittest.TestCase):
    def test_main_displays_threadx_startup_state(self):
        source = (ROOT / "Core/Src/main.c").read_text()
        self.assertIn("led_off(LED1_PORT, LED1_PIN);", source)
        self.assertIn("led_on(LED2_PORT, LED2_PIN);", source)

    def test_thread_creation_failures_have_unique_stage_patterns(self):
        source = (ROOT / "Core/Src/app_threadx.c").read_text()
        for stage in range(1, 7):
            self.assertIn(f"startup_fault({stage}U);", source)
        self.assertIn("blink(Blue, false, (fi32)stage);", source)
        self.assertIn("blink(Green, true, 1);", source)

    def test_telemetry_startup_has_success_and_failure_indicators(self):
        source = (ROOT / "Core/Src/telemetry.c").read_text()
        self.assertIn("if (init_telemetry_router() != SEDS_OK)", source)
        self.assertIn("blink(Blue, true, 3);", source)
        self.assertIn("blink(Green, false, 2);", source)
        self.assertIn("led_off(LED2_PORT, LED2_PIN);", source)
