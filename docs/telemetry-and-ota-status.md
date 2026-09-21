# Telemetry logging and OTA status

SD append operations capture network/local timestamps before taking the SD
buffer lock. They use a nonblocking try-lock so a high-priority telemetry
callback cannot spin on a preempted lower-priority SD writer. Contention increments
`g_sd_lock_contention_drops`; it drops a log record, not a flight/network task.
The producer changes buffers only when the other buffer is available, avoiding
overwriting an in-progress SD write. Host tests compile these production functions.

These fixes address demonstrated lock-order and buffer-ownership defects. They
do not establish the cause of an observed 30–40-second telemetry pause where
commands still work. Full-system/hardware validation remains required.

## OTA layout migration

Live SEDSNet delta OTA now uses stream port 4510 with 112-byte chunks and 16-byte
flash-write alignment. The application validates and stages the patch; LaunchCore
installs it at reboot and the application confirms the boot after five seconds.
An interrupted stream aborts staging and can be retried without overwriting the
running application.

**One initial wired factory-image flash is required.** An older FC bootloader
cannot install this layout migration through OTA. Do not flash only the new
application onto the old layout.

| Region | Address | Size |
|---|---|---|
| Bootloader | `0x08000000` | 8 KiB |
| Application header | `0x08002000` | 512 B |
| Application vectors/code | `0x08002200` | 455.5 KiB |
| Reversible delta staging | `0x08074000` | 16 KiB |
| Metadata copies | `0x08078000`, `0x0807A000` | 8 KiB each |
| Persistent settings (unchanged) | `0x0807C000` | 16 KiB |

The polling-only bootloader uses a core-exception vector table with external
interrupts masked. LaunchCore installs the application's full CubeMX vector table
and restores normal interrupt operation at handoff. Bootloader C code uses LTO
and size optimization; linker limits must remain enforced.

Build the combined image with `./build.py release`; build an OTA artifact with
`./build.py release ota`. The latter uses the previous packaged image as its
base when available. The installed base must match. **16 KiB is a tight limit:**
only small deltas fit, and many code changes will require a wired update. When
the patch cannot fit or no base is available, the script emits a full-image
`.seds` recovery artifact, which GroundStation deliberately rejects for live OTA.
FC has no UART bootloader recovery transport; use a wired factory flash for
that fallback.

FC telemetry uses SEDSNet's byte-payload API for its native little-endian numeric
payloads, preserving schema-based routing and fixed-length padding while avoiding
unused generic numeric conversion code. Both logging and SD support remain enabled.

`./build.py test` includes receiver framing/alignment, reset delay, interrupted
upload retry and partition checks. With Release dependencies present it also
compiles the production LaunchCore installer against simulated H523 flash,
installs/confirms a small delta and checks persistent settings are untouched.
These tests do not replace an initial wired-flash and hardware OTA qualification.

The Release factory image also passed the remote instruction-level boot check
with all configured memory thresholds. This short isolated-board check is not
a network OTA transfer or a long-duration multi-board soak.
