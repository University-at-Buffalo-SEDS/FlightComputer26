import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIXED_LAUNCHCORE = "59255a784c31cc53be2c1ee276d0d94224ed13f0"
UNSAFE_HANDOFFS = {
    "1ab6cd3dcddb7acaacb9dbfc16159f36f19363a8",
    "709474c68b83d259ba8657038340577ed4e8c6e4",
}


class LaunchCoreHandoffContract(unittest.TestCase):
    def test_bootloader_uses_stack_safe_application_handoff(self):
        cmake = (ROOT / "cmake/launchcore_stm32.cmake").read_text()
        self.assertIn(f"GIT_TAG {FIXED_LAUNCHCORE}", cmake)
        for unsafe_revision in UNSAFE_HANDOFFS:
            self.assertNotIn(unsafe_revision, cmake)

    def test_memory_report_uses_bsp_ram_capacity(self):
        cmake = (ROOT / "cmake/launchcore_stm32.cmake").read_text()
        self.assertIn("LAUNCHCORE_INTERNAL_SRAM_SIZE _launchcore_total_ram", cmake)
        self.assertIn('RAM_CAPACITY "${_launchcore_total_ram}"', cmake)

    def test_underglow_uses_launchcore_managed_persistent_storage(self):
        source = (ROOT / "Core/Src/av_bay_underglow.c").read_text()
        storage = (ROOT / "Bootloader/storage_internal_flash.c").read_text()
        cmake = (ROOT / "cmake/launchcore_stm32.cmake").read_text()
        self.assertIn('#include "launchcore/persist.h"', source)
        self.assertIn("launchcore_storage_set_driver", source)
        self.assertIn("launchcore_persist_get", source)
        self.assertIn("launchcore_persist_set", source)
        self.assertIn(".persistent_data_write_size=16u", storage)
        self.assertIn('bootloader/src/persist.c"', cmake)

    def test_flight_buzzer_uses_the_same_persistent_network_variable_path(self):
        source = (ROOT / "Core/Src/flight_buzzer.c").read_text()
        telemetry = (ROOT / "Core/Src/telemetry.c").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn("SEDS_DT_FLIGHT_BUZZER", source)
        self.assertIn("launchcore_persist_get", source)
        self.assertIn("launchcore_persist_set", source)
        self.assertIn("HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin", source)
        self.assertIn("flight_buzzer_init(r)", telemetry)
        self.assertIn("flight_buzzer_poll(g_router.r)", telemetry)
        self.assertIn("Core/Src/flight_buzzer.c", cmake)


if __name__ == "__main__":
    unittest.main()
