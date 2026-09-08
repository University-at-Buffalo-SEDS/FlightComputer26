import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def user_block(source: str, name: str) -> str:
    match = re.search(
        rf"/\* USER CODE BEGIN {re.escape(name)} \*/(.*?)/\* USER CODE END {re.escape(name)} \*/",
        source,
        flags=re.DOTALL,
    )
    if match is None:
        raise AssertionError(f"missing CubeMX USER CODE block: {name}")
    return match.group(1)


class CubeMxRegenerationContracts(unittest.TestCase):
    def test_ioc_preserves_user_code_and_owns_threadx_pool_size(self):
        ioc = (ROOT / "FlightComputer26.ioc").read_text(encoding="utf-8")
        self.assertIn("ProjectManager.KeepUserCode=true", ioc)
        self.assertIn("ProjectManager.TargetToolchain=CMake", ioc)
        self.assertIn("THREADX.TX_APP_MEM_POOL_SIZE=86016", ioc)

    def test_boot_critical_board_hooks_are_in_preserved_blocks(self):
        main = (ROOT / "Core" / "Src" / "main.c").read_text(encoding="utf-8")
        includes = user_block(main, "Includes")
        startup = user_block(main, "2")
        self.assertIn('#include "resilient_storage.h"', includes)
        self.assertIn("#define HAL_SD_Init flight_sd_init", includes)
        self.assertIn("av_bay_underglow_restore();", startup)
        self.assertIn("flight_buzzer_restore();", startup)
        self.assertIn("flight_state_cache_restore();", startup)

    def test_filex_retry_policy_is_user_owned(self):
        filex = (ROOT / "FileX" / "App" / "app_filex.c").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "#define fx_media_open flight_fx_media_open",
            user_block(filex, "Includes"),
        )
        self.assertIn("#undef fx_media_open", user_block(filex, "Includes"))
        self.assertTrue((ROOT / "Core" / "Src" / "resilient_storage.c").is_file())

    def test_cmake_reconnects_board_code_and_hash_after_generation(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        launchcore = (ROOT / "cmake" / "launchcore_stm32.cmake").read_text(
            encoding="utf-8"
        )
        self.assertIn("Core/Src/resilient_storage.c", cmake)
        self.assertIn("HAL_HASH_MODULE_ENABLED", cmake)
        self.assertIn("HAL_HASH_MODULE_ENABLED", launchcore)
        self.assertIn("add_subdirectory(cmake/stm32cubemx)", cmake)


if __name__ == "__main__":
    unittest.main()
