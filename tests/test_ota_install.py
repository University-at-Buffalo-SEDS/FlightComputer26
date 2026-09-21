"""Exercise LaunchCore's real delta installer with the FC flash geometry."""
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
import tempfile
import unittest
import build

ROOT = Path(__file__).resolve().parents[1]


class FlightOtaTests(unittest.TestCase):
    def test_delta_cannot_be_written_with_a_raw_flash_tool(self):
        with self.assertRaisesRegex(SystemExit, "Delta OTA files cannot be flashed directly"):
            build.flash(Path("update.seds"), "", {})

    def test_layout_and_startup_are_consistent(self):
        layout = json.loads((ROOT / "sim/board.json").read_text())["memory"]
        self.assertEqual(layout["bootloader_size"], 8192)
        self.assertEqual(layout["slot_a_base"], 0x08002000)
        self.assertEqual(layout["slot_a_base"] + layout["slot_a_size"], layout["delta_base"])
        self.assertEqual(layout["delta_base"] + layout["delta_size"], 0x08078000)
        self.assertEqual(layout["persistent_data_base"], 0x0807C000)
        linker = (ROOT / "STM32H523xx_FLASH.ld").read_text()
        self.assertIn("ORIGIN = 0x08002200,  LENGTH = 0x71E00", linker)
        startup = (ROOT / "Bootloader/startup.c").read_text()
        self.assertIn('"cpsid i"', startup)
        self.assertIn("boot_vectors[16]", startup)
        self.assertIn("VECT_TAB_OFFSET=0U", (ROOT / "cmake/launchcore_stm32.cmake").read_text())
        self.assertIn("ota_stream_init(r)", (ROOT / "Core/Src/telemetry.c").read_text())
        self.assertIn("ota_stream_poll();", (ROOT / "Core/Src/telemetry.c").read_text())

    def test_delta_install_retry_and_confirmation_preserve_settings(self):
        core = ROOT / "build/Release/_deps/sedslaunchcore-src"
        if not (core / "tools/mkdelta.py").exists():
            self.skipTest("Run ./build.py release to fetch LaunchCore for the installer test")
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            # Use the actual linked FC payload when present. Only its package
            # version changes, so this is a bounded small-delta qualification,
            # not a claim that every firmware change fits in 16 KiB.
            binary = ROOT / "build/Release/FlightComputer26.bin"
            payload = binary.read_bytes() if binary.exists() else (
                struct.pack("<II", 0x20033000, 0x08002209) + b"\0" * 2040)
            raw = temp / "firmware.bin"
            raw.write_bytes(payload)
            for name, version in [("base", "1.0.0"), ("target", "1.0.1")]:
                subprocess.run([sys.executable, str(core / "tools/mkimage.py"),
                    "--input", str(raw), "--output", str(temp / f"{name}.img"),
                    "--slot-base", "0x08002000", "--vector-table", "0x08002200",
                    "--header-size", "0x200", "--version", version, "--xip"], check=True)
            subprocess.run([sys.executable, str(core / "tools/mkdelta.py"),
                "--base", str(temp / "base.img"), "--target", str(temp / "target.img"),
                "--output", str(temp / "update.seds"), "--erase-size", "0x2000",
                "--slot-size", "0x72000", "--delta-slot-size", "0x4000", "--force"], check=True)
            sources = [core / "bootloader/src" / f"{name}.c" for name in
                       ["delta", "crc32", "metadata", "image_validate", "sha256"]]
            sources += [core / "update_lib/src" / f"{name}.c" for name in
                        ["delta_update", "confirm_boot"]]
            exe = temp / "ota-install"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "Bootloader"), "-I", str(core / "bootloader/include"),
                "-I", str(core / "update_lib/include"),
                str(ROOT / "tests/ota_install_test.c"),
                str(ROOT / "Bootloader/storage_dispatch.c"),
                *map(str, sources), "-o", str(exe)], check=True)
            subprocess.run([str(exe), str(temp / "base.img"),
                            str(temp / "update.seds"), str(temp / "target.img")], check=True)


if __name__ == "__main__":
    unittest.main()
