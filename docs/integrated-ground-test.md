# Integrated ground test image

The full design is in GroundStation26 `docs/integrated-ground-test.md`. This
page covers what lives in the FC.

## Build

The ground-test code is compiled only into a dedicated image. Build it with:

```sh
python3 build.py build --release --ground-test
```

This passes `-DINTEGRATED_GROUND_TEST=ON`, which defines `FC_GROUND_TEST=1`.
Every other build passes OFF explicitly. A flight image contains none of the
replay code: each hook in the sources sits under `#if FC_GROUND_TEST`, and
`tests/test_ground_test.py` checks this. The flight image reports no
ground-test support, so the station refuses the IGT commands.

Sizes, Release, against dev a602b51:

| Image | FLASH | RAM |
| --- | --- | --- |
| flight (default) | +96 B | +0 B |
| `--ground-test` | +6,604 B | +3,392 B |

The flight image's 96 B are the `SIM_SAMPLE` schema entry, the four compat
command slots and one SD event label.

## Pieces

| File | Role |
| --- | --- |
| `Core/Src/ground_test.c`, `Core/Inc/ground_test.h` | Mode, CSV validation and parsing, rings, interpolation, gate state |
| `Core/Src/sdpipeline.c` | Calls `igt_sd_service()`: the SD task owns every IGT FileX call |
| `Core/Src/dma.c` | Replaces fresh IMU/baro samples while the replay runs; the real raw rows still go to SD |
| `Core/Src/distribution.c` | Replayed GPS and the rail swap; `SIM_SAMPLE` for replayed IMU/baro; real values on the normal types |
| `Core/Src/evaluation.c` | `SIM_SAMPLE` for estimator outputs during the replay; Landed ends it |
| `Core/Src/recovery.c` | `IGT_Enter`, `IGT_Live_Pyro`, `IGT_Dry_Run`, `IGT_Exit` (compat 29–32); replay start at Launch acceptance |
| `Core/Src/deployment_pulse.c` | Dry-run gate; every Stop/reset path calls `igt_stop()` |
| `Core/Src/fc_watchdog_recovery.c` | No checkpoint while the mode is on, so a reset cannot resume a replayed flight |
| `Core/Src/board_health.c` | BOARD_HEALTH word 14 bits 4..15 |

## SD card

The CSVs go in `/IGT/` (for example `IMU0.CSV`, `BARO0.CSV`, `GPS0.CSV`). The
schema is in the GS doc, section 4. To generate or convert the files, use
GroundStation26 `tools/igt_csv.py`.
