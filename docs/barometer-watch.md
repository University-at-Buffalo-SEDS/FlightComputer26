# ST-Link sensor and FC estimator viewer

`tools/watch_barometer.py` reads the FC's barometer, three accelerometer axes, and three gyroscope axes directly over ST-Link. Normal observation works without firmware changes or SEDSnet telemetry. Updated normal firmware includes automatic monitor mode and FC estimator control; no build feature flags are required. CSV replay requires no probe or OpenOCD. The viewer needs Python 3.10+; live reads also need OpenOCD 0.12+. Its browser plots use Python's standard library and Canvas. STM32CubeProgrammer CLI is used only for optional USB probe listing/automatic serial selection. Passing `--serial` avoids that dependency.

```sh
# Inspect symbol addresses and flash coverage without accessing any probe:
python3 tools/watch_barometer.py --build build/Release --inspect

# List attached probes without attaching to a CPU:
python3 tools/watch_barometer.py --list-probes

# Automatically find the local build matching the flashed application:
./tools/watch_barometer.py

# Or explicitly select the exact flashed application ELF/build:
python3 tools/watch_barometer.py --build build/Release --serial YOUR_STLINK_SERIAL
python3 tools/watch_barometer.py --elf /path/to/FlightComputer26.elf --rate 1 --history 1800 --csv barometer.csv
```

With no ELF/build argument, the viewer checks local `build/*/FlightComputer26.elf` files, newest first, and uses one only after its initialized flash sections match the target. `--inspect` without a build argument describes the newest local ELF without accessing the board. Connection failures include the captured OpenOCD output so USB access failures, target power problems, and probe contention can be distinguished.

Use `--openocd /path/to/openocd` if it is not on PATH. For USB listing only, use `--programmer /path/to/STM32_Programmer_CLI` if needed. Multiple probes require `--serial`; the viewer never selects one arbitrarily. A busy probe produces an error, without killing an existing debugger or taking over its connection. `--no-browser` prints the local viewer URL for manual opening. The server binds only to `127.0.0.1`; on a Pi accessed over SSH, use an SSH tunnel and `--port 8765 --no-browser`.

The viewer keeps one OpenOCD ST-Link DAP connection open on the H523's Cortex-M33 **AP1**. It loads no board/flash configuration, disables GDB/telnet, and uses a private Tcl RPC port bound to `127.0.0.1`. Small RAM reads are batched into one request per snapshot without temporary files or reconnecting the probe. Initial flash verification uses binary dumps for compatibility with OpenOCD 0.12's large-response limits. The process and connection close when the viewer exits. With compatible firmware, connecting automatically requests monitor mode by writing only the command word of its versioned mailbox; it never edits flight configuration, task state, peripheral registers, or outputs. Do not use a different debugger to halt/reset the board during an observation.

On the tested H523/STLINK-V3MINI with CubeProgrammer 2.21.0, connecting with the previous HOTPLUG reader left external NVIC interrupts disabled: uptime continued through SysTick while barometer/DMA acquisition stopped. OpenOCD preserved interrupt delivery. If an old monitor has already caused this state, stop it and reset/power-cycle the FC once before starting this version. Exiting an admitted monitor session resets the board automatically.

Before interpreting RAM, every initialized flash section in the selected ELF is compared byte-for-byte with target flash. Initialized `.data` is verified at its **flash load address**, not its live RAM address. ELF headers, nonloaded padding, NOBITS/BSS and the separately generated LaunchCore image header are excluded. This verifies the selected application's loaded code/data, not bootloader or LaunchCore metadata. Mismatch refuses sampling. `--allow-mismatched-firmware` is an explicit diagnostic override; it visibly marks the resulting view unverified and may interpret the wrong RAM. Normally select the correct ELF instead.

Addresses and sizes come from the selected ELF's symbol table, including supported LTO local-symbol names. A stripped or incompatible build fails clearly. The viewer supports the current FC float-only `measm` (48 bytes) and optional `kf_svec` prediction (36 bytes) layouts, with a runtime source-header field-order guard and compiled field-offset regression checks. Use the ELF built from this checkout: Release ELF symbols supply addresses/sizes, but do not contain member-level DWARF to prove a different historical ABI. Size mismatch refuses access. Future reordered layouts require updating the script/tests rather than guessing.

Plots show compensated sensor pressure/temperature, measured altitude, the driver's altitude bias and pad pressure where those symbols survive optimization. “Before bias” is explicitly derived from driver altitude + bias. Kalman altitude/vertical velocity are the available **intermediate prediction**, not final corrected state. Raw ADC counts are not retained as readable symbols and are not invented. Optional optimized-away values display **Unavailable**.

Gyroscope values are the firmware's degrees/second. Accelerometer values are shown in **driver units**, preserving the configured scale including `SCALE_HEURISTIC` rather than silently applying another conversion. These channels come directly from `meas`, not a UI filter. Per-axis values are best-effort live reads; the existing firmware ABI does not expose independent IMU sample-age timestamps.

## FC firmware Kalman controls

Build and flash the updated normal firmware once, using your usual build/flash settings. Monitor support is always included; no special build profile or feature flag is needed. Then connect normally:

```sh
python3 tools/watch_barometer.py --build build/Release
```

The monitor automatically requests bench mode after verifying the flashed firmware, regardless of the current flight state. The recovery owner first inhibits flight sequencing, cancels active deployment pulses, and cooperatively stops the flight evaluator at its safe barrier. The bench estimator waits for that acknowledgement before accessing the shared Kalman state. While monitor mode is active, deployment commands remain inhibited. Older firmware remains observable with controls unavailable until it is updated. `--read-only` opts out of monitor mode and the exit reset; a firmware-identity mismatch override also disables control.

The filter starts stopped. Use **Start FC Kalman**, **Stop FC Kalman**, and **Reset FC Kalman** in the page. Reset clears cycle counters and reinitializes a running filter from fresh measurements; a stopped filter stays stopped until Start. `--enable-kalman` optionally starts it immediately. These controls run the FC's ascent prediction/update functions and publish corrected altitude, velocity, and quaternion channels alongside the raw sensor plots. Firmware sensor filtering is unchanged; the UI applies no filtering.

The backend renews a two-second monitor lease, including while the estimator is stopped or the requested sample rate is slow. **Ctrl-C exits monitor mode and the firmware resets itself into normal operation.** Killing the backend, losing the probe connection, or missing the heartbeat also triggers a firmware reset after the lease expires. The monitor hold remains in force until reset. Closing a browser tab does not end the backend session. A reset returns through normal boot and preserves the existing recovery/arming rules; it does not automatically launch or arm the FC.

## Saving and replaying

**Save retained CSV** downloads the samples currently retained by the live viewer. `--csv session.csv` continuously records every accepted monitor sample for the whole session, including IMU channels and any available FC estimates; writes flush as rows arrive. Files are created exclusively, so existing recordings are never overwritten. Stopping/resetting the estimator does not stop sensor acquisition or recording.

Open a saved recording without hardware:

```sh
python3 tools/watch_barometer.py --replay session.csv
```

Use the replay's **Play/Pause**, **Restart**, speed selector, and Seek slider. Expanded graph zoom/pan remain available. Replay uses the recorded values and recorded FC estimates directly; it does not rerun a filter or enable hardware controls. It validates elapsed time ordering, supports older recordings with absent channels shown as unavailable, and loads the recording into memory. CSV export during replay preserves the complete loaded recording, including samples after the playhead.

Reads happen while the CPU is running and are **not an atomic firmware snapshot**. Two grouped reads are taken; when the barometer acquisition tick advances between them, the newer read is accepted. Requiring identical samples would discard normal acquisition because uploads can take longer than a sensor period. If the tick stays the same but barometer fields change, the viewer retries, up to three attempts, before discarding the sample as inconsistent. Unrelated IMU and monotonic tick changes are allowed. Changing prediction data is omitted for that sample. This check cannot detect all tearing across independently read fields. Nonfinite/invalid pressure, halted CPU and short reads are discarded. A stopped acquisition tick becomes stale; MCU uptime restarting stops sampling and requires a new flash verification. The viewer never resumes a halted CPU.

The default sampling request is **20 Hz**, configurable with `--rate` from 0.1 to 50 Hz. Actual throughput depends on SWD and host performance. The browser refreshes at 10 Hz and requests only new history rows; old cursors automatically recover the retained history. `/data` also reports `read_duration_ms` and `host_sample_age_ms` to separate probe read time from time since the latest accepted sample. These are host measurements, not full sensor-to-screen latency. The firmware's 25 Hz barometer configuration and sensor IIR filtering still limit physical response; increasing the viewer rate does not change flight acquisition or filtering. For slow drift recordings, use `--rate 1` as in the example above.

The UI applies **no averaging, smoothing, or point reduction**. It connects every valid retained value in the visible time range directly, with gaps for stale/missing data. The pressure/temperature channels are the firmware's compensated sensor readings; plotting them directly does not undo the sensor's own IIR filter or turn them into ADC counts. Firmware filtering is unchanged.

Click **Expand** or a card's graph to inspect a larger plot. Choose a window from 30 seconds through 1 hour, or all retained data. Scroll over the expanded graph to zoom around the pointer; drag horizontally or use the Pan slider to move through history. **Pause view** freezes the displayed time range while acquisition continues; **Go live** returns to the latest readings. Hover to inspect the nearest recorded sample. The summary reports count, min/max, first-to-last change, and a clearly labeled linear drift estimate in units/hour. The fit is a numerical summary only and never replaces or filters the plotted values. Graph times are elapsed since the monitor started.

Default retention is **one hour** at the default 20 Hz request (72,000 samples). `--history` changes the retention request, capped at 100,000 samples; use a lower `--rate` for longer recordings. The page shows the actual retained interval. CSV is opt-in, uses exclusive creation, and records accepted values with host monotonic elapsed seconds, FC uptime and barometer sample tick (milliseconds), sample age and staleness. It does not overwrite existing recordings. Stop with Ctrl-C; an admitted monitor session resets the MCU into normal boot.

Offline tests (no hardware connection):

```sh
python3 -m unittest discover -s tests -p 'test_watch_barometer.py'
FC_BAROSCOPE_ELF=/path/to/FlightComputer26.elf python3 -m unittest discover -s tests -p 'test_watch_barometer.py'
# Optional real RPC transport test using OpenOCD's dummy adapter, no hardware:
FC_BAROSCOPE_RPC_TEST=1 python3 -m unittest discover -s tests -p 'test_watch_barometer.py'
```
