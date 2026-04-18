# Flight Computer 26

This is the UB SEDS Flight Computer repository for IREC 2026.

## Functionality

The Flight Computer is responsible for maintaining an accurate record of rocket states during flight and collecting data for post-mortem flight reconstruction. A state is described by the flight stage name (e.g., Burnout, Descent) and a history of last N state vectors, which record relative height, velocity, and position. An accurate finite state machine enables the rocket to take autonomous actions on state transitions. 

If telemetry is enabled, the Flight Computer is also responsible for logging flight events, reporting errors, and processing incoming messages. Otherwise, all logs will be redirected to a device behind `stdout`, with message processing limited to inter-thread communication.

## Structure

The main logic resides in [Core/](/Core/) and is split across source, includes, and tests. The main logic is organized into tasks managed by the ThreadX scheduler. Each task comprises of application code that depends on common API, abstracted through platform and domain-specific headers, which in turn can be poisoned when necesasry (e.g., when testing or swapping hardware).

Domain-specific part of this year's Flight Computer relies on the following assumptions:

* The board includes Bosch Sensortec BMP390 barometer and BMI088 inertial measurement unit, connected to a single SPI bus, with interrupt pins (at least 1 per sensor) being connected to free GPIO pins of the microcontroller.

* GPS device measurements are delivered over CAN bus from an external board. If telemetry is disabled or the data becomes unavailable during flight, then BMP390 barometer will be used as fallback.

* The board includes optional SD card (connected via SDMMC and managed by USBX) and LEDs.

* The Flight Computer is the single master on the board.

## Dependencies

* Newlib Cygwin (comes with toolchain)
* STM32H5 HAL Driver (bundled, 1)
* ARM CMSIS (bundled, 1)
* ThreadX (2), FileX, and USBX (bundled, 1)
* sedsprintf_rs (optional) (submodule, 3)
* GCC ARM EABI (make)
* CMake (make)
* Python (optional, for build script)

(1) Please see [NOTICE](/NOTICE) on licensing information and compliance.

(2) Both Eclipse ThreadX and Microsoft Azure RTOS can be used interchangeably.

(3) If a different library is preferred, the new library must implement the API used in [telemetry.c](/Core/Src/telemetry.c), or telemetry can be disabled during compilation.

## Building

Use [build.py](/build.py) to build, flash, configure, toggle tests, benchmarks, and debug options, and to generate assembly code for the repository. The script includes a short manual page on its usage, along with the description of available options.

To use [sedsprintf_rs](https://github.com/Rylan-Meilutis/sedsprintf_rs), install [Rust](https://rust-lang.org/), initialize the submodule with `git submodule update --init --recursive`, and follow the instructions in the library's repository. Please note that this library is distributed under the terms of GNU General Public License 2.0.

## IREC 2026

If you are a UB SEDS team member, you can join the [OpenProject board](https://projects.rylanswebsite.com/projects/avionics-2025) for this year.