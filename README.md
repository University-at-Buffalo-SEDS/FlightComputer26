# FlightComputer26 firmware

FlightComputer26 targets the STM32H523. ThreadX tasks acquire the BMI088 IMU
and BMP390 barometer, maintain the flight state, log data, and exchange
telemetry and commands over the avionics CAN-FD network.

CMake fetches SEDSNet v4.0.18 and SEDS LaunchCore v1.0.0 without submodules.
LaunchCore generates linker scripts from `Bootloader/board_config.h`, packages
the application, and owns OTA and persistent-storage formats. The underglow,
startup-buzzer, and flight-state managed variables are restored from persistent
storage before network synchronization and updated when an authoritative
network value arrives.

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

## Tests

```sh
python3 build.py test
python3 build.py test --all --release
```

The full suite builds release factory and OTA artifacts, validates the STM32H523
flash and SRAM regions, runs GoogleTest and Python units, boots the ELF with
simulated IMU/barometer/flash/CAN devices, profiles allocator use, and verifies
linked SEDSNet discovery, time sync, persistent variables, and command/ACK
traffic. Hardware behavior and probe limits are declared in `sim/board.json`.

The board's runtime network schema is `config/sedsnet.json`; changes to network
variables or endpoints must update that file and the matching stable C IDs.
