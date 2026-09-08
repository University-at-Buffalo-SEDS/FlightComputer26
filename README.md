# FlightComputer26 firmware

FlightComputer26 targets the STM32H523. ThreadX tasks acquire the BMI088 IMU
and BMP390 barometer, maintain the flight state, log data, and exchange
telemetry and commands over the avionics CAN-FD network.

CMake fetches SEDSNet v4.0.20 and SEDS LaunchCore v1.0.0 without submodules.
LaunchCore generates linker scripts from `Bootloader/board_config.h`, packages
the application, and owns OTA and persistent-storage formats. The underglow,
startup-buzzer, and flight-state managed variables are restored from persistent
storage before network synchronization and updated when an authoritative
network value arrives. The buzzer record also retains the authoritative packet
timestamp so a reliable update left in flight before a reset cannot roll the
restored state back after reboot; legacy one-byte records remain readable.

## Build and flash

The legacy build interface uses positional options:

```sh
python3 build.py release factory
python3 build.py release flash-st
python3 build.py release flash-dfu
python3 build.py release clean
```

`factory` builds the combined bootloader and confirmed Slot A image;
`bootloader`, `firmware`, and `ota` select the other image workflows. A normal
blank-board flash must use the complete factory image at `0x08000000`.

## Regenerating with STM32CubeMX

Open `FlightComputer26.ioc`, allow CubeMX to update the selected STM32CubeH5
package when desired, and choose **Generate Code** with the CMake toolchain.
The `.ioc` owns the 86,016-byte ThreadX application pool. Board startup,
persistent-variable restoration, and resilient optional-SD handling live in
preserved `USER CODE` blocks plus `Core/Src/resilient_storage.c`; the top-level
CMake project reconnects all board-owned sources, LaunchCore, SEDSNet, the
relocated application image, and hardware HASH after generation.

After every regeneration, run `python3 build.py test --full --release`. The
static regeneration contracts fail if CubeMX drops a protected hook, and the
release/simulator stages catch link-layout, startup, allocator, peripheral,
isolated-CAN, factory-boot, and OTA regressions.

## Tests

```sh
python3 build.py test
python3 build.py test --all --release
```

The full suite builds release factory and OTA artifacts, validates the STM32H523
flash and SRAM regions, runs GoogleTest and Python units, boots the ELF with
simulated IMU/barometer/flash/CAN devices, profiles allocator use, and verifies
linked SEDSNet discovery, time sync, persistent variables, and command/ACK
traffic. The disconnected-CAN stage must observe recoverable H5 TX backpressure
while the telemetry loop advances with zero allocator, panic, startup, or
unexpected queue errors. Hardware behavior and probe limits are declared in
`sim/board.json`.

The board's runtime network schema is `config/sedsnet.json`; changes to network
variables or endpoints must update that file and the matching stable C IDs.
