# Read-only ST-Link barometer viewer

`tools/watch_barometer.py` reads the FC's running RAM directly over ST-Link. It works without SEDSnet telemetry and adds no firmware or flash usage. It needs Python 3.10+ and OpenOCD 0.12+ on macOS or Linux/Ubuntu; its local browser plots use only Python's standard library and Canvas. STM32CubeProgrammer CLI is used only for optional USB probe listing/automatic serial selection. Passing `--serial` avoids that dependency.

```sh
# Inspect symbol addresses and flash coverage without accessing any probe:
python3 tools/watch_barometer.py --build build/Release --inspect

# List attached probes without attaching to a CPU:
python3 tools/watch_barometer.py --list-probes

# Select the exact currently flashed application ELF/build:
python3 tools/watch_barometer.py --build build/Release --serial YOUR_STLINK_SERIAL
python3 tools/watch_barometer.py --elf /path/to/FlightComputer26.elf --rate 1 --history 1800 --csv barometer.csv
```

Use `--openocd /path/to/openocd` if it is not on PATH. For USB listing only, use `--programmer /path/to/STM32_Programmer_CLI` if needed. Multiple probes require `--serial`; the viewer never selects one arbitrarily. A busy probe produces an error, without killing an existing debugger or taking over its connection. `--no-browser` prints the local viewer URL for manual opening. The server binds only to `127.0.0.1`; on a Pi accessed over SSH, use an SSH tunnel and `--port 8765 --no-browser`.

The viewer uses OpenOCD's ST-Link DAP driver and the H523's Cortex-M33 **AP1** for memory dumps. It loads no board/flash configuration and disables OpenOCD's GDB/Tcl/telnet servers. It never requests reset, halt, resume, programming, debug-freeze, watchdog changes, or output commands. Do not use a different debugger to halt/reset the board during an observation.

On the tested H523/STLINK-V3MINI with CubeProgrammer 2.21.0, connecting with the previous HOTPLUG reader left external NVIC interrupts disabled: uptime continued through SysTick while barometer/DMA acquisition stopped. OpenOCD preserved interrupt delivery. If an old monitor has already caused this state, stop it and reset/power-cycle the FC once before starting this version. The viewer does not reset the board automatically.

Before interpreting RAM, every initialized flash section in the selected ELF is compared byte-for-byte with target flash. Initialized `.data` is verified at its **flash load address**, not its live RAM address. ELF headers, nonloaded padding, NOBITS/BSS and the separately generated LaunchCore image header are excluded. This verifies the selected application's loaded code/data, not bootloader or LaunchCore metadata. Mismatch refuses sampling. `--allow-mismatched-firmware` is an explicit diagnostic override; it visibly marks the resulting view unverified and may interpret the wrong RAM. Normally select the correct ELF instead.

Addresses and sizes come from the selected ELF's symbol table, including supported LTO local-symbol names. A stripped or incompatible build fails clearly. The viewer supports the current FC float-only `measm` (48 bytes) and optional `kf_svec` prediction (36 bytes) layouts, with a runtime source-header field-order guard and compiled field-offset regression checks. Use the ELF built from this checkout: Release ELF symbols supply addresses/sizes, but do not contain member-level DWARF to prove a different historical ABI. Size mismatch refuses access. Future reordered layouts require updating the script/tests rather than guessing.

Plots show compensated sensor pressure/temperature, measured altitude, the driver's altitude bias and pad pressure where those symbols survive optimization. “Before bias” is explicitly derived from driver altitude + bias. Kalman altitude/vertical velocity are the available **intermediate prediction**, not final corrected state. Raw ADC counts are not retained as readable symbols and are not invented. Optional optimized-away values display **Unavailable**.

Reads happen while the CPU is running and are **not an atomic firmware snapshot**. Two grouped reads are taken; when the barometer acquisition tick advances between them, the newer read is accepted. Requiring identical samples would discard normal acquisition because uploads can take longer than a sensor period. If the tick stays the same but barometer fields change, the viewer retries, up to three attempts, before discarding the sample as inconsistent. Unrelated IMU and monotonic tick changes are allowed. Changing prediction data is omitted for that sample. This check cannot detect all tearing across independently read fields. Nonfinite/invalid pressure, halted CPU and short reads are discarded. A stopped acquisition tick becomes stale; MCU uptime restarting stops sampling and requires a new flash verification. The viewer never resumes a halted CPU. ST-Link/OpenOCD overhead limits actual sampling rate; the default one read/second suits slow drift. `--rate` requests 0.1–10 Hz and does not change firmware acquisition.

History is capped at 20,000 samples; gaps/stale readings break graph lines. CSV is opt-in, uses exclusive creation, and records accepted values with host monotonic elapsed seconds, FC uptime and barometer sample tick (milliseconds), sample age and staleness. It does not overwrite existing recordings. Stop with Ctrl-C; the MCU is left as found.

Offline tests (no hardware connection):

```sh
python3 -m unittest discover -s tests -p 'test_watch_barometer.py'
FC_BAROSCOPE_ELF=/path/to/FlightComputer26.elf python3 -m unittest discover -s tests -p 'test_watch_barometer.py'
```
