#!/usr/bin/env python3
import subprocess
from pathlib import Path
import sys
import os


def run(cmd: list[str]) -> None:
    print("Running:", " ".join(cmd))
    subprocess.run(cmd, check=True)


def main() -> None:
    repo_root = Path(__file__).parent.resolve()
    os.chdir(repo_root)
    project_dir = Path.cwd()
    build_dir = project_dir / "build" / "HostDebug"
    telemetry_flag = f"-DENABLE_TELEMETRY=OFF"

    build_dir.mkdir(parents=True, exist_ok=True)

    run([
        "cmake",
        "-DCMAKE_BUILD_TYPE=Debug",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
        "-DTEST_ON_HOST=ON",
        telemetry_flag,
        "-S", str(project_dir),
        "-B", str(build_dir),
        "-G", "Ninja",
    ])

    run([
        "cmake",
        "--build", str(build_dir),
        "--target", "fctest",
        "--parallel",
    ])


if __name__ == "__main__":
    main()
