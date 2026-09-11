import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class StackGuardContracts(unittest.TestCase):
    def test_command_worker_has_measured_hardware_headroom(self):
        tasks = (ROOT / "Core" / "Inc" / "fctasks.h").read_text()
        self.assertIn("RECV_STACK_BYTES (12U * 1024U)", tasks)

    def test_threadx_stack_guard_reports_the_offending_thread(self):
        config = (ROOT / "Core" / "Inc" / "tx_user.h").read_text()
        app = (ROOT / "Core" / "Src" / "app_threadx.c").read_text()
        self.assertRegex(config, r"(?m)^#define TX_ENABLE_STACK_CHECKING$")
        self.assertIn("tx_thread_stack_error_notify(thread_stack_error_handler)", app)
        self.assertIn("g_thread_stack_error_thread", app)
        self.assertIn("startup_fault(7U)", app)

    def test_simulator_rejects_stack_guard_events(self):
        layout = (ROOT / "sim" / "board.json").read_text()
        self.assertIn('"symbol": "g_thread_stack_error_count", "maximum": 0', layout)
        self.assertIn('"symbol": "g_recovery_stack_remaining", "minimum": 2048', layout)

    def test_command_worker_reports_stack_headroom(self):
        recovery = (ROOT / "Core" / "Src" / "recovery.c").read_text()
        self.assertIn("_tx_thread_stack_analyze(&recovery_task)", recovery)
        self.assertIn("g_recovery_stack_remaining", recovery)
        self.assertIn("recovery_update_stack_profile();", recovery)

    def test_hardfault_diagnostics_are_required_by_the_simulator(self):
        handler = (ROOT / "Core/Src/stm32h5xx_it.c").read_text()
        layout = (ROOT / "sim/board.json").read_text()
        self.assertIn("hardfault_capture_and_halt", handler)
        self.assertIn("core_frame += 18U", handler)
        self.assertIn('"symbol": "g_hardfault_count", "maximum": 0', layout)
        self.assertIn('"symbol": "g_hardfault_cfsr"', layout)

    def test_build_output_distinguishes_simulator_instrumentation(self):
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn("Simulator instrumentation: ${SEDS_FIRMWARE_SIM_TEST}", cmake)
        self.assertIn("Simulation user config:    ${SWAP_CONFIG}", cmake)


if __name__ == "__main__":
    unittest.main()
