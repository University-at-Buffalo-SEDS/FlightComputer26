import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import build


class OtaBuildScriptTests(unittest.TestCase):
    def test_all_tests_preserve_the_selected_build_mode(self):
        self.assertEqual(build.parse_test_options(["--all"]), (True, False))
        self.assertEqual(
            build.parse_test_options(["--all", "--release"]), (True, True)
        )
        self.assertEqual(build.parse_test_options(["--full"]), (True, False))

    def test_simulation_layout_uses_selected_build_directory(self):
        from sim.run_full import load_layout_for_build

        root = Path(build.__file__).resolve().parent
        layout = load_layout_for_build(root, "Selected_Test_Build")
        for artifact in layout["artifacts"].values():
            self.assertEqual(Path(artifact).parts[1], "Selected_Test_Build")

    def test_all_tests_report_an_unavailable_docker_daemon(self):
        from sim import run_full

        probe = mock.Mock(returncode=1, stdout="", stderr="daemon unavailable")
        with mock.patch.object(run_full.shutil, "which", return_value="/usr/bin/docker"):
            with mock.patch.object(run_full.subprocess, "run", return_value=probe):
                with self.assertRaisesRegex(RuntimeError, "daemon is not available"):
                    run_full.require_docker()

    def test_dfu_flash_defaults_to_combined_factory_image(self):
        _preset, options = build.parse(["release", "flash-dfu"])
        self.assertEqual(options["image"], "factory")

    def test_dfu_flash_leaves_rom_bootloader_after_download(self):
        image = Path("FlightComputer26.factory.bin")
        options = {"flash-dfu": True, "flash-st": False, "flash-stlink": False}
        dfu_output = ("Download [=========================] 100%\r"
                      "File downloaded successfully\nSubmitting leave request...\n"
                      "dfu-util: Error during download get_status\n")
        process = mock.Mock(stdout=io.BytesIO(dfu_output.encode("utf-8")))
        process.wait.return_value = 74
        visible_output = io.StringIO()
        with mock.patch.object(Path, "exists", return_value=True):
            with mock.patch.object(build, "require_tool", return_value="dfu-util"):
                with mock.patch.object(build.subprocess, "Popen", return_value=process) as popen:
                    with contextlib.redirect_stdout(visible_output):
                        build.flash(image, "0x08000000", options)
        command = popen.call_args.args[0]
        self.assertEqual(command[4], "0x08000000:leave")
        self.assertEqual(
            visible_output.getvalue(),
            dfu_output.replace("dfu-util: Error during download get_status\n", ""),
        )

    def test_ota_option_is_available(self):
        _preset, options = build.parse(["release", "ota"])
        self.assertEqual(options["image"], "ota")

    def test_bsp_ota_layout_detection(self):
        cases = {
            "delta-macro": ("delta", "#define BOARD_DELTA_SIZE 0x6000u\n", "", 0x6000),
            "ab": ("ab", "#define BOARD_SLOT_B_SIZE 0x40000u\n", "", 0x40000),
            "delta-slot-b": (
                "delta",
                "#define BOARD_SLOT_B_SIZE 0x8000u\n",
                "static const int layout = {.slot_b_is_delta = true};\n",
                0x8000,
            ),
            "staging": ("staging", "#define BOARD_APP_STAGING_SIZE 0x50000u\n", "", 0x50000),
            "recovery": ("recovery", "#define BOARD_SLOT_A_SIZE 0x70000u\n", "", 0),
        }
        for label, (expected, config, storage, expected_size) in cases.items():
            with self.subTest(label), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                (root / "Bootloader").mkdir()
                (root / "Bootloader" / "board_config.h").write_text(
                    config, encoding="utf-8"
                )
                (root / "Bootloader" / "storage_internal_flash.c").write_text(
                    storage, encoding="utf-8"
                )
                with mock.patch.object(build, "PROJECT", root):
                    mode, size = build.detect_ota_layout()
                self.assertEqual(mode, expected)
                self.assertEqual(size, expected_size)

    def test_ota_is_a_full_recovery_seds_image(self):
        with tempfile.TemporaryDirectory() as directory:
            build_dir = Path(directory)

            def fake_build(selected_dir, _target):
                (selected_dir / "FlightComputer26.launchcore.img").write_bytes(
                    b"packaged-launchcore-image"
                )

            with mock.patch.object(build, "build", side_effect=fake_build):
                artifact, address = build.select_artifact(build_dir, "ota")

            self.assertEqual(artifact.suffix, ".seds")
            self.assertEqual(artifact.read_bytes(), b"packaged-launchcore-image")
            self.assertEqual(address, build.APP_ADDR)


if __name__ == "__main__":
    unittest.main()
