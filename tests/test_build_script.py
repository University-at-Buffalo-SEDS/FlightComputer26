import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import build


class OtaBuildScriptTests(unittest.TestCase):
    def test_clean_command_removes_the_complete_build_tree(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            build_root = root / "build"
            artifact = build_root / "Release" / "FlightComputer26.elf"
            artifact.parent.mkdir(parents=True)
            artifact.write_bytes(b"firmware")
            with mock.patch.object(build, "BUILDDIR", build_root):
                build.clean(build_root)
            self.assertFalse(build_root.exists())

    def test_clean_rejects_paths_outside_build_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            build_root = root / "build"
            outside = root / "outside"
            outside.mkdir()
            with mock.patch.object(build, "BUILDDIR", build_root):
                with self.assertRaisesRegex(SystemExit, "outside"):
                    build.clean(outside)
            self.assertTrue(outside.exists())

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
        expected_mcus = {
            "stm32g4": "stm32g491",
            "stm32h5": "stm32h523",
            "stm32u5": "stm32u585",
        }
        self.assertEqual(layout["mcu"], expected_mcus[layout["architecture"]])
        ioc = next(root.glob("*.ioc")).read_text(encoding="utf-8")
        expected_parts = {
            "stm32g491": "Mcu.CPN=STM32G491",
            "stm32h523": "Mcu.CPN=STM32H523",
            "stm32u585": "Mcu.CPN=STM32U585",
        }
        self.assertIn(expected_parts[layout["mcu"]], ioc)
        for artifact in layout["artifacts"].values():
            self.assertEqual(Path(artifact).parts[1], "Selected_Test_Build")

    def test_all_tests_report_an_unavailable_docker_daemon(self):
        from sim import run_full

        probe = mock.Mock(returncode=1, stdout="", stderr="daemon unavailable")
        with mock.patch.object(run_full.shutil, "which", return_value="/usr/bin/docker"):
            with mock.patch.object(run_full.subprocess, "run", return_value=probe):
                with self.assertRaisesRegex(RuntimeError, "daemon is not available"):
                    run_full.require_docker()

    def test_missing_registry_image_is_built_from_a_fresh_clone(self):
        from sim import run_full

        missing = mock.Mock(returncode=1, stdout="", stderr="denied")
        success = mock.Mock(returncode=0, stdout="", stderr="")
        ui = mock.Mock()
        with mock.patch.dict(run_full.os.environ, {}, clear=True):
            with mock.patch.object(run_full.shutil, "which", return_value="/usr/bin/git"):
                with mock.patch.object(
                    run_full.subprocess,
                    "run",
                    side_effect=[missing, missing, missing, missing, success, success],
                ) as execute:
                    image = run_full.resolve_simulator_image(
                        ui, "/usr/bin/docker", Path("/board"), "stm32g4"
                    )

        self.assertEqual(image, "seds-firmware-simulator:local-v0.4")
        commands = [call.args[0] for call in execute.call_args_list]
        self.assertIn(
            [
                "/usr/bin/docker",
                "pull",
                "ghcr.io/university-at-buffalo-seds/firmwaresimulator:latest",
            ],
            commands,
        )
        pull_call = next(
            call for call in execute.call_args_list
            if call.args[0][:2] == ["/usr/bin/docker", "pull"]
        )
        self.assertNotIn("stdout", pull_call.kwargs)
        self.assertNotIn("stderr", pull_call.kwargs)
        self.assertTrue(any("clone" in command for command in commands))
        self.assertTrue(
            any(
                command[:2] == ["/usr/bin/docker", "build"]
                and "--progress=plain" in command
                for command in commands
            )
        )
        self.assertFalse(
            any("SIM_ARCH" in argument for command in commands for argument in command)
        )
        self.assertFalse(
            any("--platform" in command for command in commands)
        )

    def test_generated_layout_is_readable_by_the_container_user(self):
        from sim.run_full import write_container_layout

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            layout = write_container_layout(root, {"name": "test"})
            self.assertEqual(root.stat().st_mode & 0o777, 0o755)
            self.assertEqual(layout.stat().st_mode & 0o777, 0o644)

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
