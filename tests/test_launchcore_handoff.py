import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIXED_LAUNCHCORE = "7cdbca87b7fad2c72d73257f4fb1b14df6b280a0"
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
        persistent_store = (ROOT / "Core/Src/persistent_store.c").read_text()
        storage = (ROOT / "Bootloader/storage_internal_flash.c").read_text()
        cmake = (ROOT / "cmake/launchcore_stm32.cmake").read_text()
        self.assertIn('#include "persistent_store.h"', source)
        self.assertIn("launchcore_storage_set_driver", persistent_store)
        self.assertIn("launchcore_persist_get", persistent_store)
        self.assertIn("launchcore_persist_set", persistent_store)
        self.assertIn("NETWORK_VARIABLE_REFRESH_INTERVAL_MS", source)
        self.assertIn(".persistent_data_write_size=16u", storage)
        self.assertIn('bootloader/src/persist.c"', cmake)

    def test_underglow_is_restored_before_threadx_and_network_sync(self):
        main = (ROOT / "Core/Src/main.c").read_text()
        restore = main.index("av_bay_underglow_restore();")
        self.assertLess(restore, main.index("MX_ThreadX_Init();", restore))
        source = (ROOT / "Core/Src/av_bay_underglow.c").read_text()
        self.assertIn("if (g_restore_attempted) return;", source)

    def test_flight_buzzer_uses_the_same_persistent_network_variable_path(self):
        source = (ROOT / "Core/Src/flight_buzzer.c").read_text()
        telemetry = (ROOT / "Core/Src/telemetry.c").read_text()
        cmake = (ROOT / "CMakeLists.txt").read_text()
        self.assertIn("SEDS_DT_FLIGHT_BUZZER", source)
        self.assertIn("persistent_store_get", source)
        self.assertIn("persistent_store_set", source)
        self.assertIn("NETWORK_VARIABLE_REFRESH_INTERVAL_MS", source)
        self.assertIn("HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin", source)
        self.assertIn("flight_buzzer_init(r)", telemetry)
        self.assertIn("flight_buzzer_poll(g_router.r)", telemetry)
        self.assertIn("Core/Src/flight_buzzer.c", cmake)

        main = (ROOT / "Core/Src/main.c").read_text()
        restore = main.index("flight_buzzer_restore();")
        self.assertLess(restore, main.index("MX_ThreadX_Init();", restore))
        self.assertIn("if (g_restore_attempted) return;", source)

    def test_sedsnet_refresh_waits_for_a_peer_and_profiles_stack_margin(self):
        tasks = (ROOT / "Core/Inc/fctasks.h").read_text()
        config = (ROOT / "Core/Inc/fcconfig.h").read_text()
        rtos = (ROOT / "AZURE_RTOS/App/app_azure_rtos_config.h").read_text()
        telemetry = (ROOT / "Core/Src/telemetry.c").read_text()
        layout = (ROOT / "sim/board.json").read_text()
        self.assertIn("TLMT_STACK_BYTES (48U * 1024U)", tasks)
        self.assertIn("TELEMETRY_HEAP", config)
        self.assertIn("48U * 1024U", config)
        self.assertIn("69632U + (16U * 1024U)", rtos)
        self.assertIn("g_telemetry_discovery_seen != 0U", telemetry)
        self.assertIn("g_telemetry_stack_free_min", telemetry)
        self.assertIn("telemetry_sample_stack_margin();", telemetry)
        self.assertIn("telemetry_sample_active_stack_margin();", telemetry)
        self.assertIn("__get_PSP()", telemetry)
        self.assertIn('"symbol": "g_telemetry_stack_free_min"', layout)
        self.assertIn('"minimum": 8192', layout)


if __name__ == "__main__":
    unittest.main()
