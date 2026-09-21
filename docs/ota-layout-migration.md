# Flight Computer delta OTA

GroundStation can upload a delta over SEDSNet stream port 4510. The receiver
stages 112-byte chunks with 16-byte flash alignment; LaunchCore installs the
patch on restart and the application confirms the boot after five seconds.
An interrupted upload aborts staging and can be retried.

**Install the new combined factory image once over a wired programmer.** The
old bootloader cannot install this flash-layout migration by OTA. Do not put
the new application alone onto the old layout.

| Region | Address | Size |
|---|---|---|
| Bootloader | `0x08000000` | 8 KiB |
| Application header | `0x08002000` | 512 B |
| Application vectors/code | `0x08002200` | 455.5 KiB |
| Delta staging | `0x08074000` | 16 KiB |
| Metadata copies | `0x08078000`, `0x0807A000` | 8 KiB each |
| Persistent settings (unchanged) | `0x0807C000` | 16 KiB |

Build the factory image with `./build.py release`; use
`./build.py release flash-st` for a wired flash. Build an OTA artifact with
`./build.py release ota`. Its previous packaged base must match the installed
image. Only small deltas fit in 16 KiB; larger changes require a wired update.
When no base exists or a delta does not fit, the script emits a full-image
`.seds` recovery artifact. GroundStation rejects that artifact for live OTA.
FC has no UART bootloader recovery transport; use a wired factory flash instead.

The polling-only bootloader masks external interrupts and uses a compact
core-exception vector table, LTO, and size optimization. LaunchCore installs
the application's complete vector table and restores interrupts at handoff.
Application telemetry uses native little-endian byte payloads to avoid unused
generic numeric conversion code without removing SD logging or schema routing.

`./build.py test` covers receiver framing, alignment, interrupted-upload retry,
reset delay, and partition limits. With Release dependencies available, it also
compiles the production LaunchCore installer against simulated H523 flash,
installs and confirms a small delta, and checks persistent settings survive.
The remote instruction-level factory boot and memory checks passed. This short
isolated check is not an end-to-end network OTA or a long-duration soak;
wired migration and hardware OTA still need hardware qualification.
