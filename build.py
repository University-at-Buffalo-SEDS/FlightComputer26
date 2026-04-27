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

OPTIONS: (any option not specified -> opposite is true)

	flash-dfu       Download executable to eabi target.
                        Prereq: dfu-utils.

        flash-st        Alternative to flash-dfu over STLink.
                        Prereq: STM32_Programmer_CLI over SWD.

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

        exspin          For linked libraries, use wakeyield spinlock
                        instead of TX mutex. Experimental.
"""

from __future__ import annotations

import sys
import os
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
                        "exspin",
                }

# Repo constants
PROJECT         = Path(__file__).parent.resolve()
BUILDDIR        = PROJECT / "build"
BIN             = "FlightComputer26.bin"
ELF             = "FlightComputer26.elf"
FC_ADDR         = "0x08000000"
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

        return preset, options


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

        if options["notelemetry"] or preset == "Debug":
                telem = "-DENABLE_TELEMETRY=OFF"
                gps = "-DEXTERNAL_GPS=OFF"
                sd = "-DONBOARD_SD=OFF"

                if options["sensortest"]:
                        sensortest = "-DSENSOR_TESTS=ON"
        else:
                if options["nogps"]:
                        gps = "-DEXTERNAL_GPS=OFF"
                if options["nosd"]:
                        sd = "-DONBOARD_SD=OFF"

        if options["nousb"]:
                        usb = "-DUSB_ENUM=OFF"

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

        if options["exspin"]:
                exspin = "-DEXPORT_SPINLOCK=ON"

        cmake_args = [
                "cmake",
                f"-DCMAKE_BUILD_TYPE={preset}",
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake",
                telem,
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
                "-S", str(PROJECT),
                "-B", str(buildir),
                "-G", "Ninja",
        ]

        run(cmake_args)


def build(buildir: Path):
        run([
                "cmake",
                "--build",
                str(buildir),
                "--parallel"
        ])


def objcopy(buildir: Path) -> Path:
        elf_path = buildir / ELF
        bin_path = buildir / BIN

        if not elf_path.exists():
                sys.exit(f"Expected ELF at {elf_path}")

        run([
                "arm-none-eabi-objcopy",
                "-O", "binary",
                str(elf_path),
                str(bin_path),
        ])

        return bin_path


def flash(path: Path, options: dict):
        if not path.exists():
                sys.exit(f"Expected BIN at {path}")

        cmd = []

        if options["flash-dfu"]:
                require_tool("dfu-util")
                cmd = [ "dfu-util", "-a", "0",
                        "-s", FC_ADDR, "-D", str(path),
                ]
        elif options["flash-st"]:
                stm32prog = require_tool(STM32_PROG_CLI)
                cmd = [ stm32prog,
                        "-c", "port=SWD", "mode=UR", "reset=HWrst",
                        "-w", str(path), FC_ADDR,
                        "-v",
                        "-rst",
                ]

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
        if path.exists():
                shutil.rmtree(path)
        else:
                sys.exit(f"No such directory: {path}")

        path.mkdir(parents=True, exist_ok=True)


def asmgen(buildir: Path):
        path = buildir / f"{PROJECT.name}.asm"

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


def main() -> None:
        os.chdir(PROJECT)
        preset, options = parse(sys.argv[1:])
        buildir = BUILDDIR / preset

        if options["clean"]:
                clean(buildir)
                return

        configure(buildir, preset, options)

        if options["configure"]:
                return

        build(buildir)

        if options["asm"]:
                asmgen(buildir)

        executable = objcopy(buildir)

        if options["flash-dfu"] or options["flash-st"]:
                flash(executable, options)
        elif preset == "Release":
                return
        elif options["stlink"]:
                gdb_st_session(executable)


if __name__ == "__main__":
        main()
