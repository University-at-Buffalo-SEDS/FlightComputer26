#!/usr/bin/env python3

"""
Unified build script.

This script is used to streamline building
the UB SEDS 2026 Flight Computer executable.

Build prerequisites:
        Python >= 3.11
        CMake >= 3.22
        GCC ARM Embedded ABI >= 13.3.1

Usage:
        `python build.py [PRESET] [OPTIONS]`

PRESET:
        debug   - includes debug info
	release	- enables optimizations

If none specified, defaults to Debug.

OPTIONS:
	flash           - download executable to eabi
	                - target (requires dfu-utils)

	notelemetry     - disable telemetry, redirect 
                        - output to terminal emulator;
                        - disables message handling

        clean           - cleans build folder for the
                        - specified preset (or default);
                        - option has highest precedence

        dmatest         - enables local DMA testing
                        - does not start ThreadX
                        - prerequisite: notelemetry

        fullcmd         - expect full FC commands in
                        - handler and not byte codes;
                        - prerequisite: telemetry

        batching        - handle potentially several
                        - batched messages in handler;
                        - prerequisite: telemetry

        configure       - configure specified preset
                        - with given options, but do
                        - not build the project

        nogps           - absence of external GPS device;
                        - always use Ascent Kalman filter;
                        - True if telemetry is disabled

        nosd            - absence of on-board SD card;
                        - True if telemetry is disabled

        asm             - generate assembly code for the
                        - preset and options chosen
			
If an option is not specified, then either it is not in effect
or its complement (default per CMakeLists.txt) is in effect.
"""

from __future__ import annotations

import sys
import os
import shutil
import subprocess
import importlib.util
from pathlib import Path


# Defaults
DEFAULT_PRESET  = "Debug"

# Configuration
ALL_PRESETS     = {"debug" : "Debug", "release" : "Release"}
ALL_OPTIONS     = {"flash", "notelemetry", "clean", "dmatest", "fullcmd",
                        "batching", "configure", "nogps", "nosd", "asm"}

# Repo constants
PROJECT         = Path(__file__).parent.resolve()
BUILDDIR        = PROJECT / "build"
BIN             = "FlightComputer26.bin"
ELF             = "FlightComputer26.elf"


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
                print(f"Command failed: {e}")
                sys.exit(1)


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

        # Defaults for IREC 2026
        batch   = "-DMESSAGE_BATCHING=OFF"
        telem   = "-DENABLE_TELEMETRY=ON"
        compat  = "-DTELEMETRY_COMPAT=ON"
        dmatest = "-DDMA_TESTING=OFF"
        gps     = "-DEXTERNAL_GPS=ON"
        sd      = "-DONBOARD_SD=ON"

        if options["notelemetry"]:
                telem = "-DENABLE_TELEMETRY=OFF"
                gps = "-DEXTERNAL_GPS=OFF"
                sd = "-DONBOARD_SD=OFF"
                if options["dmatest"]:
                        dmatest = "-DDMA_TESTING=ON"
        else:
                if options["fullcmd"]:
                        compat = "-DTELEMETRY_COMPAT=OFF"
                if options["batching"]:
                        batch  = "-DMESSAGE_BATCHING=ON"
                if options["nogps"]:
                        gps = "-DEXTERNAL_GPS=OFF"
                if options["nosd"]:
                        sd = "-DONBOARD_SD=OFF"

        cmake_args = [
                "cmake",
                f"-DCMAKE_BUILD_TYPE={preset}",
                "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
                telem,
                dmatest,
                batch,
                compat,
                gps,
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


def flash(path: Path):
        if not path.exists():
                sys.exit(f"Expected BIN at {path}")

        run([
                "dfu-util",
                "-a", "0",
                "-s", "0x08000000",
                "-D", str(path),
        ])


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

        if options["flash"]:
                flash(executable)


if __name__ == "__main__":
        main()