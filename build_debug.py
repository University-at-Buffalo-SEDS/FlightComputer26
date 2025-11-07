#!/usr/bin/env python3
import subprocess
from pathlib import Path


def run(cmd: list[str]) -> None:
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main() -> None:
    project_dir = Path.cwd()
    build_dir = project_dir / "build" / "Debug"

    # 1) Configure with CMake
    run([
        "cmake",
        f"-DCMAKE_BUILD_TYPE=debug",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake",
        "-DCMAKE_COMMAND=cmake",
        "-S", str(project_dir),
        "-B", str(build_dir),
        "-G", "Ninja",
    ])

    # 2) Build
    run([
        "cmake",
        "--build", str(build_dir),
        "--parallel",
    ])

    # 3) objcopy ELF → BIN
    elf_path = build_dir / "FlightComputer26.elf"
    bin_path = build_dir / "FlightComputer26.bin"

    run([
        "arm-none-eabi-objcopy",
        "-O", "binary",
        str(elf_path),
        str(bin_path),
    ])


if __name__ == "__main__":
    main()
