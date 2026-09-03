#!/usr/bin/env python3

"""
This script is used to streamline building
the UB SEDS 2026 Flight Computer executable.

Prerequisites:  Python >= 3.11
                CMake >= 3.22
                GCC ARM EABI >= 13.3.1

Usage:  `python build.py [PRESET] [OPTIONS]`

PRESET: debug   - default, with debug symbols (O0 or Og)
	release	- strips debug symbols (O3 or O3 + LTO)

OPTIONS: (option not specified -> opposite is true)

	flash-dfu       Download executable to eabi target.
                        Prereq: dfu-utils.

        flash-st        Alternative to flash-dfu over STLink.
                        Prereq: STM32_Programmer_CLI over SWD.

        flash-stlink    Flash with the open-source st-flash utility.

        factory         Build bootloader + packaged firmware (default).
        bootloader      Build only the bootloader.
        firmware        Build only the packaged Slot A firmware.
        ota             Build a full .seds image for bootloader recovery.

        stlink          Open STLink connection and exit.
                        Prereq: Debug.

        notelemetry     Disable telemetry and message handling.
                        Output logs to stdout (unless nousb).
                        Default option for Debug.

        clean           Cleans build folder for specified preset.
                        This option has highest precedence.

        fullcmd         Expect full 4-byte commands in handler.
                        Otherwise expect 1-byte codes.

        batching        In handler, dispatch potentially several
                        batched messages at once. 

        parallelkf      Enable parallel predict/update support.
                        Partially implemented, deferred until 2027.

        configure       Configure for specified preset and exit.

        nogps           No external GPS device. Rely on Barometer
                        during descent. True if notelemetry.

        nosd            No on-board SD card. True if notelemetry.

        asm             Generate assembly for specified options.

        bench           Compile with project-wide benchmarks.

        userflags       Use flags in CMakeLists for building.
                        Debug: O0 -> Og. Release: += LTO.

        sensortest      Run synchronous (polling) sensor tests.
                        Prereq: notelemetry.

        nousb           Do not assume correctly enumerating USB.
                        Prereq: notelemetry

        gmath           Report file and line for failures of math
                        functions using underlying API's status.

        lunatic         Bypass flight state checks when executing
                        mission-crtitical commands. For testing.

        simulation      Swap user config to simulation one with
                        adjusted thesholds.

        manualconfirm   Prompt the user to confirm critical actions
                        within a specified timeout. Recommended.

        exspinlock      For linked libraries, use wakeyield spinlock
                        instead of TX mutex. Experimental.

        alloctest       Test exported allocator in Core/Src/fchooks.c.
"""

from __future__ import annotations

import sys
import io
import os
import re
import time
import socket
import shutil
import subprocess
import importlib.util
from pathlib import Path


# Defaults
DEFAULT_PRESET  = "Debug"

# Configuration
ALL_PRESETS     = {"debug" : "Debug", "release" : "Release"}
ALL_OPTIONS     = {     "flash-dfu",
                        "flash-st",
                        "flash-stlink",
                        "stlink",
                        "notelemetry", 
                        "clean",
                        "fullcmd",
                        "batching",
                        "configure",
                        "nogps",
                        "nosd",
                        "asm",
                        "bench",
                        "userflags",
                        "sensortest",
                        "nousb",
                        "parallelkf",
                        "gmath",
                        "lunatic",
                        "simulation",
                        "manualconfirm",
                        "exspinlock",
                        "alloctest",
                        "fakestation",
                        "factory",
                        "bootloader",
                        "firmware",
                        "ota",
                }

# Repo constants
PROJECT         = Path(__file__).parent.resolve()
BUILDDIR        = PROJECT / "build"
FIRMWARE_NAME   = "FlightComputer26"
BIN             = "FlightComputer26.bin"
ELF             = "FlightComputer26.elf"
FC_ADDR         = "0x08000000"
APP_ADDR        = "0x08004000"
DEBUG_HOST      = "127.0.0.1"
DEBUG_PORT      = 4242
STM32_PROG_CLI  = "STM32_Programmer_CLI"


def run(cmd: list[str], *, pipeline: bool = False):
        try:
                if pipeline:
                        proc = subprocess.run(
                                cmd,
                                check=True,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True
                        )
                        return proc
                else:        
                        subprocess.run(cmd, check=True)
                        return None

        except Exception as e:
                sys.exit(f"Command failed: {e}")


def require_tool(name: str) -> str:
        path = shutil.which(name)

        if path is None:
                sys.exit(f"Required tool not found in PATH: {name}")

        return path


def parse(argv: list[str]):
        preset = DEFAULT_PRESET
        options = {opt: False for opt in ALL_OPTIONS}

        for a in argv:
                arg = a.lower()

                if arg in ALL_PRESETS:
                        preset = ALL_PRESETS[arg]
                        continue
                
                if arg in ALL_OPTIONS:
                        options[arg] = True
                        continue

                sys.exit(f"Unrecognized option: {a}")

        images = [name for name in ("factory", "bootloader", "firmware", "ota")
                  if options[name]]
        if len(images) > 1:
                sys.exit(f"Select only one image type, not: {', '.join(images)}")
        options["image"] = images[0] if images else "factory"
        return preset, options


def board_macro_optional(suffix: str) -> int | None:
        text = (PROJECT / "Bootloader" / "board_config.h").read_text(encoding="utf-8")
        matches = re.findall(
                rf"(?m)^#define\s+[A-Za-z0-9_]*{re.escape(suffix)}\s+"
                rf"(0x[0-9A-Fa-f]+|[0-9]+)u?\s*$", text
        )
        if len(matches) > 1:
                sys.exit(f"Expected at most one BSP macro ending in {suffix}")
        return int(matches[0], 0) if matches else None


def detect_ota_layout() -> tuple[str, int]:
        storage = "\n".join(
                source.read_text(encoding="utf-8", errors="replace")
                for source in sorted((PROJECT / "Bootloader").glob("*.c"))
        )
        delta_size = board_macro_optional("DELTA_SIZE")
        slot_b_size = board_macro_optional("SLOT_B_SIZE")
        staging_size = board_macro_optional("APP_STAGING_SIZE")
        slot_b_is_delta = re.search(
                r"\.slot_b_is_delta\s*=\s*true\b", storage
        ) is not None
        if delta_size:
                return "delta", delta_size
        if slot_b_size:
                return ("delta" if slot_b_is_delta else "ab"), slot_b_size
        if staging_size:
                return "staging", staging_size
        return "recovery", 0


def configure(buildir: Path, preset: str, options: dict):
        buildir.mkdir(parents=True, exist_ok=True)

        # Defaults for IREC 2026 (except compilation flags)
        # Debug preset will forcefully disable telemetry
        # and things that depend on it.
        batch           = "-DMESSAGE_BATCHING=OFF"
        telem           = "-DENABLE_TELEMETRY=ON"
        compat          = "-DTELEMETRY_COMPAT=ON"
        gps             = "-DEXTERNAL_GPS=ON"
        sd              = "-DONBOARD_SD=ON"
        bench           = "-DFC_BENCH=OFF"
        flags           = "-DCUSTOM_FLAGS=OFF"
        sensortest      = "-DSENSOR_TESTS=OFF"
        usb             = "-DUSB_ENUM=ON"
        parkf           = "-DPARALLEL_KF=OFF"
        mathdbg         = "-DDEBUG_MATH=OFF"
        lunatic         = "-DIGNORE_STATES=OFF"
        simul           = "-DSWAP_CONFIG=OFF"
        manualconfirm   = "-DMANUAL_CONFIRM=OFF"
        exspin          = "-DEXPORT_SPINLOCK=OFF"
        alloctest       = "-DALLOC_TEST=OFF"
        fakestation     = "-DFAKESTATION=OFF"

        if options["notelemetry"] or preset == "Debug":
                telem = "-DENABLE_TELEMETRY=OFF"
                gps = "-DEXTERNAL_GPS=OFF"

                if options["sensortest"]:
                        sensortest = "-DSENSOR_TESTS=ON"
                if options["fakestation"]:
                        fakestation = "-DFAKESTATION=ON"
        else:
                if options["nogps"]:
                        gps = "-DEXTERNAL_GPS=OFF"
                if options["alloctest"]:
                        alloctest = "-DALLOC_TEST=ON"

        if options["nousb"]:
                        usb = "-DUSB_ENUM=OFF"

        if options["nosd"]:
                        sd = "-DONBOARD_SD=OFF"

        if options["fullcmd"]:
                compat = "-DTELEMETRY_COMPAT=OFF"

        if options["batching"]:
                batch  = "-DMESSAGE_BATCHING=ON"

        if options["parallelkf"]:
                parkf = "-DPARALLEL_KF=ON"

        if options["bench"]:
                bench = "-DFC_BENCH=ON"

        if options["gmath"]:
                mathdbg = "-DDEBUG_MATH=ON"

        if options["userflags"]:
                flags = "-DCUSTOM_FLAGS=ON"

        if options["lunatic"]:
                lunatic = "-DIGNORE_STATES=ON"

        if options["simulation"]:
                simul = "-DSWAP_CONFIG=ON"

        if options["manualconfirm"]:
                manualconfirm   = "-DMANUAL_CONFIRM=ON"

        if options["exspinlock"]:
                exspin = "-DEXPORT_SPINLOCK=ON"

        cmake_args = [
                "cmake",
                f"-DCMAKE_BUILD_TYPE={preset}",
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake",
                telem,
                "-DSEDS_FIRMWARE_SIM_TEST=" + ("ON" if os.environ.get("SEDS_FIRMWARE_SIM_TEST") == "1" else "OFF"),
                batch,
                compat,
                gps,
                sd,
                bench,
                flags,
                sensortest,
                usb,
                parkf,
                mathdbg,
                lunatic,
                simul,
                manualconfirm,
                exspin,
                alloctest,
                fakestation,
                "-S", str(PROJECT),
                "-B", str(buildir),
                "-G", "Ninja",
        ]

        run(cmake_args)


def build(buildir: Path, target: str):
        run([
                "cmake",
                "--build",
                str(buildir),
                "--target", target,
                "--parallel"
        ])


def objcopy(buildir: Path, elf_name: str) -> Path:
        elf_path = buildir / elf_name
        bin_path = elf_path.with_suffix(".bin")

        if not elf_path.exists():
                sys.exit(f"Expected ELF at {elf_path}")

        run([
                "arm-none-eabi-objcopy",
                "-O", "binary",
                str(elf_path),
                str(bin_path),
        ])

        return bin_path


def select_artifact(buildir: Path, image: str) -> tuple[Path, str]:
        ota_layout, ota_secondary_size = detect_ota_layout()
        automatic_base = None
        if image == "ota" and ota_layout == "delta":
                previous = buildir / f"{FIRMWARE_NAME}.launchcore.img"
                if previous.exists():
                        automatic_base = buildir / f".{FIRMWARE_NAME}.ota-base.launchcore.img"
                        shutil.copy2(previous, automatic_base)
        target = {
                "factory": "factory-image",
                "bootloader": f"{FIRMWARE_NAME}Bootloader",
                "firmware": FIRMWARE_NAME,
                "ota": FIRMWARE_NAME,
        }[image]
        build(buildir, target)
        if image == "factory":
                path, address = buildir / f"{FIRMWARE_NAME}.factory.bin", FC_ADDR
        elif image in ("firmware", "ota"):
                path, address = buildir / f"{FIRMWARE_NAME}.launchcore.img", APP_ADDR
        else:
                path = objcopy(buildir, f"{FIRMWARE_NAME}Bootloader.elf")
                address = FC_ADDR
        if not path.exists():
                sys.exit(f"Expected {image} artifact at {path}")
        if image == "ota":
                ota_path = buildir / f"{FIRMWARE_NAME}.seds"
                if automatic_base is not None:
                        delta_tool = buildir / "_deps" / "sedslaunchcore-src" / "tools" / "mkdelta.py"
                        erase_size = board_macro_optional("FLASH_ERASE_SIZE")
                        slot_size = board_macro_optional("SLOT_A_SIZE")
                        if delta_tool.exists() and erase_size and slot_size:
                                delta = subprocess.run([
                                        sys.executable, str(delta_tool),
                                        "--base", str(automatic_base), "--target", str(path),
                                        "--output", str(ota_path), "--erase-size", hex(erase_size),
                                        "--slot-size", hex(slot_size),
                                        "--delta-slot-size", hex(ota_secondary_size),
                                ], capture_output=True, text=True)
                                automatic_base.unlink(missing_ok=True)
                                if delta.returncode == 0:
                                        print("Built reversible-delta OTA from the BSP delta layout.")
                                        return ota_path, ""
                if automatic_base is not None:
                        automatic_base.unlink(missing_ok=True)
                shutil.copy2(path, ota_path)
                path = ota_path
                route = {"ab": "A/B slot", "staging": "single-slot staging",
                         "recovery": "bootloader recovery", "delta": "bootloader recovery"}[ota_layout]
                print(f"Built full-image OTA for {route}.")
        print(f"Built {image} image: {path} (flash address {address})")
        return path, address


def run_dfu(cmd: list[str]):
        proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                bufsize=0,
        )
        output_chunks = []
        line_chunks = []
        deferred_status = ""
        assert proc.stdout is not None
        stream = io.TextIOWrapper(
                proc.stdout, encoding="utf-8", errors="replace", newline=""
        )
        while text := stream.read(1):
                output_chunks.append(text)
                line_chunks.append(text)
                if text in ("\r", "\n"):
                        line = "".join(line_chunks)
                        line_chunks.clear()
                        if "dfu-util: Error during download get_status" in line:
                                deferred_status += line
                        else:
                                print(line, end="", flush=True)
        if line_chunks:
                line = "".join(line_chunks)
                if "dfu-util: Error during download get_status" in line:
                        deferred_status += line
                else:
                        print(line, end="", flush=True)
        returncode = proc.wait()
        output = "".join(output_chunks)
        reset_disconnect = (
                returncode == 74
                and "File downloaded successfully" in output
                and "Submitting leave request" in output
                and "Error during download get_status" in output
        )
        if reset_disconnect:
                return
        if deferred_status:
                print(deferred_status, end="", flush=True)
        if returncode != 0:
                sys.exit(f"Command failed (exit {returncode}): dfu-util")


def flash(path: Path, address: str, options: dict):
        if not path.exists():
                sys.exit(f"Expected BIN at {path}")

        cmd = []

        if options["flash-dfu"]:
                require_tool("dfu-util")
                dfuse_address = address if ":" in address else f"{address}:leave"
                cmd = [ "dfu-util", "-a", "0",
                        "-s", dfuse_address, "-D", str(path),
                ]
        elif options["flash-st"]:
                stm32prog = require_tool(STM32_PROG_CLI)
                cmd = [ stm32prog,
                        "-c", "port=SWD", "mode=UR", "reset=HWrst",
                        "-w", str(path), address,
                        "-v",
                        "-rst",
                ]
        elif options["flash-stlink"]:
                stflash = require_tool("st-flash")
                cmd = [stflash, "--reset", "write", str(path), address]

        if options["flash-dfu"]:
                run_dfu(cmd)
        else:
                run(cmd)


def gdb_st_session(path: Path):
        try:
                subprocess.Popen(["st-util"],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        start_new_session=True
                )
        except Exception as e:
                sys.exit(f"St-util failed: {e}")
        
        timeout = time.time() + 5.0

        while time.time() < timeout:
                try:
                        sock = socket.create_connection(
                                (DEBUG_HOST, DEBUG_PORT),
                                timeout=0.5
                        )
                        sock.close()
                        break
                except Exception:
                        time.sleep(0.1)
        else:
                sys.exit("STLink connection failed")

        print("You can now attach gdb and/or end debugging session")
        # For example:
        # gdb build/Debug/FlightComputer26.bin
        # (gdb) target extended-remote 127.0.0.1:4242
        # killall st-util


def clean(path: Path):
        build_root = BUILDDIR.resolve()
        target = path.resolve()
        try:
                target.relative_to(build_root)
        except ValueError:
                sys.exit(f"Refusing to clean outside the build directory: {target}")

        if not target.exists():
                print(f"Already clean: {target}")
                return
        if target.is_symlink():
                sys.exit(f"Refusing to recursively clean a symbolic link: {target}")

        shutil.rmtree(target)
        print(f"Cleaned build artifacts: {target}")


def asmgen(buildir: Path):
        path = buildir / f"{FIRMWARE_NAME}.asm"

        elf_path = buildir / ELF

        if not elf_path.exists():
                sys.exit(f"Expected ELF at {elf_path}")

        # Deduce assembly from ELF
        proc = run([
                "arm-none-eabi-objdump",
                "-d",
                "-S",
                str(elf_path)],
                pipeline=True
        )

        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
                f.write(proc.stdout)

        print(f"Written assembly code to {path}")


class _TestUI:
        def say(self, level: str, message: str) -> None:
                print(f"[{level}] {message}")


def run_gtests() -> None:
        source = PROJECT / "tests" / "gtest"
        cmake = shutil.which("cmake")
        if cmake is None:
                raise RuntimeError(
                        "CMake is required for GoogleTest but was not found on PATH."
                )
        build_dir = BUILDDIR / "host-gtests"
        generator = "Ninja" if shutil.which("ninja") else "Unix Makefiles"
        subprocess.run([
                cmake, "-S", str(source), "-B", str(build_dir), "-G", generator,
        ], cwd=PROJECT, check=True)
        subprocess.run([
                cmake, "--build", str(build_dir), "--parallel",
        ], cwd=PROJECT, check=True)
        subprocess.run([
                "ctest", "--test-dir", str(build_dir), "--output-on-failure",
                "--no-tests=error",
        ], cwd=PROJECT, check=True)


def print_test_summary(results: list[tuple[str, str]]) -> None:
        width = max([len("Test stage"), *(len(name) for name, _ in results)])
        print("\nFirmware test summary")
        print(f"{'Test stage':<{width}}  Result")
        print(f"{'-' * width}  ------")
        for name, result in results:
                print(f"{name:<{width}}  {result}")
        passed = sum(result == "PASS" for _, result in results)
        if passed == len(results):
                print(f"[OK] All {passed} test stages passed.")
        else:
                print(f"[ERR] {passed}/{len(results)} completed test stages passed.")


def test_failure_help(stage: str) -> str:
        if stage == "Docker readiness":
                return "Start Docker Desktop/the Docker daemon and verify 'docker info' works."
        if stage == "GoogleTest unit tests":
                return (
                        "Install CMake and a C++17 compiler. GoogleTest is found locally "
                        "or fetched automatically."
                )
        if stage == "Python unit tests":
                return "Read the first assertion above and run the named unittest directly."
        if "build" in stage.lower():
                return (
                        "Confirm the ARM GNU toolchain, CMake, Ninja, Rust, and Cargo "
                        "are on PATH; clear only the selected build cache if it is stale."
                )
        if stage == "Long-duration memory profile":
                return (
                        "Inspect the probe table for pool loss, low-water, allocation "
                        "failures, or stack errors."
                )
        if stage == "Network discovery and time sync":
                return (
                        "Check the FDCAN link and require both discovery_seen and "
                        "timesync_valid to raise network_ready."
                )
        return (
                "Review the simulator matrix for the failed row, especially memory probes, "
                "peripheral configuration, and boot/OTA artifacts."
        )


def run_test_stage(results: list[tuple[str, str]], stage: str, action) -> None:
        try:
                action()
        except Exception as exc:
                results.append((stage, "FAIL"))
                print_test_summary(results)
                detail = str(exc).strip() or type(exc).__name__
                raise RuntimeError(
                        f"{stage} failed.\n\nPossible solutions:\n"
                        f"- {test_failure_help(stage)}\n\nDetails: {detail}"
                ) from None
        results.append((stage, "PASS"))


def parse_test_options(argv: list[str]) -> tuple[bool, bool]:
        all_tests = "--all" in argv or "--full" in argv
        release = "--release" in argv
        known = {"--all", "--full", "--release"}
        unknown = [arg for arg in argv if arg not in known]
        if unknown:
                sys.exit(f"Unrecognized test option: {unknown[0]}")
        return all_tests, release


def run_tests(argv: list[str]) -> None:
        all_tests, release = parse_test_options(argv)
        results: list[tuple[str, str]] = []
        run_test_stage(results, "Python unit tests", lambda: subprocess.run([
                sys.executable, "-m", "unittest", "discover", "-s", "tests",
                "-p", "test_*.py",
        ], cwd=PROJECT, check=True))
        run_test_stage(results, "GoogleTest unit tests", run_gtests)
        if not all_tests:
                print_test_summary(results)
                return
        # Qualification needs observability probes that are intentionally
        # excluded from production/flash builds.
        os.environ["SEDS_FIRMWARE_SIM_TEST"] = "1"
        from sim.run_full import (
                require_docker,
                run_full_simulation,
                run_memory_profile,
                run_network_simulation,
        )
        run_test_stage(results, "Docker readiness", require_docker)
        preset = "release" if release else "debug"
        script = Path(__file__).resolve()
        run_test_stage(
                results, "Factory firmware build",
                lambda: subprocess.run(
                        [sys.executable, str(script), preset, "factory"], check=True
                ),
        )
        run_test_stage(
                results, "OTA package build",
                lambda: subprocess.run(
                        [sys.executable, str(script), preset, "ota"], check=True
                ),
        )
        run_test_stage(
                results, "Firmware simulation",
                lambda: run_full_simulation(
                        _TestUI(), PROJECT, "stm32h5",
                        "Release" if release else "Debug"
                ),
        )
        build_subdir = "Release" if release else "Debug"
        run_test_stage(
                results, "Long-duration memory profile",
                lambda: run_memory_profile(
                        _TestUI(), PROJECT, "stm32h5", build_subdir
                ),
        )
        run_test_stage(
                results, "Network discovery and time sync",
                lambda: run_network_simulation(
                        _TestUI(), PROJECT, "stm32h5", build_subdir
                ),
        )
        print_test_summary(results)


def main() -> None:
        os.chdir(PROJECT)
        if sys.argv[1:] == ["clean"]:
                clean(BUILDDIR)
                return
        if sys.argv[1:2] == ["test"]:
                try:
                        run_tests(sys.argv[2:])
                except RuntimeError as exc:
                        sys.exit(f"Test run failed.\n\n{exc}")
                return
        preset, options = parse(sys.argv[1:])
        buildir = BUILDDIR / preset

        if options["clean"]:
                clean(buildir)
                return

        configure(buildir, preset, options)

        if options["configure"]:
                return

        executable, address = select_artifact(buildir, options["image"])

        if options["asm"] and options["image"] != "bootloader":
                asmgen(buildir)

        if options["flash-dfu"] or options["flash-st"] or options["flash-stlink"]:
                flash(executable, address, options)
        elif preset == "Release":
                return
        elif options["stlink"]:
                gdb_st_session(executable)


if __name__ == "__main__":
        main()
