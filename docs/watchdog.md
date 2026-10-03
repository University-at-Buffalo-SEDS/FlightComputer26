# FC hardware watchdog and autonomous recovery

Build with `./build.py release --watchdog` (`watchdog` also works with the legacy CLI), or CMake
`-DENABLE_BOARD_WATCHDOG=ON`. Other boards use `build --release --watchdog`.
The default is OFF, and the build script explicitly configures ON/OFF to prevent
stale cache settings. ThreadX still schedules the firmware; this feature is
board-owned and changes no SEDSnet code.

The independent IWDG has a nominal 16.384 second deadline (LSI / 256, reload
2047). A feed requires fresh progress from network service (when enabled), the
command worker, acquisition, and evaluation, plus at least 250 ms of scheduler
progress. Preflight acquisition checks in for the dormant evaluation task;
during descent evaluation runs in acquisition. An explicit evaluation abort
also exempts the dormant evaluator. SysTick never feeds the watchdog.

## Flight restart

Completed estimator cycles and initial flight entry commit an allocation-free,
two-slot CRC-checked RAM checkpoint. A temporary ThreadX preemption threshold
protects each write from task termination; CAN and pulse interrupts remain
enabled during CRC work. It holds internal flight phase/history,
confidence/statistics, quaternion and Kalman covariance/model arrays, runtime
configuration, launch pressure/altitude baseline, GPS rail origin, and timers.
It does not retain RTOS objects, locks, pending commands, pointers to transient
buffers, or sensor DMA state. Matrix pointers are rebound to static arrays.

A watchdog reset resumes autonomously only when the retained record matches
a CRC of this full firmware image, passes CRC/layout and semantic validation, and has a valid
deployment journal and a reset gap of at most 60 seconds. RAM is in a NOLOAD
section outside startup's BSS zeroing, with a dedicated noncacheable MPU
region so journal/checkpoint writes cannot remain in a write-back cache. Rebuilding firmware invalidates old records;
this is warm-reset recovery, not power-loss persistence. Power/brownout resets
start normally and do not restore flight records.

The board reserves RTC binary mode on LSI / 32 (nominal 1 kHz) to measure elapsed
time across reset. Existing non-LSI RTC ownership is refused rather than resetting
someone else's backup domain. Flight timer ages advance across the measured gap;
estimator sample timers restart after sensors initialize, successive-sample
confidence clears, and the FSM waits for an entire fresh state history. GPS is
marked unavailable until new reports arrive, so no stale IMU sample
is integrated across the outage. LSI tolerance applies to gap timing. Network UTC
is independent of this counter.

Recovered flight bypasses preflight/postinit and ignition requests. Barometer
initialization retains launch pressure. Deployment attempts are committed to a
separate two-slot journal before GPIO assertion, so a reset between an output
pulse and the next estimator checkpoint cannot replay it. Both outputs initialize
low; a partially completed pulse is not resumed. Deployment history advances the
restored flight phase when necessary. Repeated ordinary deployment commands do
not repeat a completed pulse; the explicit force/test path retains its prior role.

Missing/corrupt/stale records or a failed recovery clock inhibit launch and
actuation after a watchdog reset. Debug globals report rejection, resume, gap,
checkpoint count, reset flags, and missing progress bits. Recovery from that
inhibited condition requires inspection and a normal power restart; there is no
silent fallback to another ignition attempt.

## Installation and validation

Install the matching bootloader before enabling IWDG. It continues running across
a warm reset; the board bootloader services it during initialization, storage
operations, and application handoff. Use a wired factory image for bootloader
updates. An old bootloader may reset repeatedly before reaching the application.

Native tests exercise all resumable flight phases, CRC corruption, interrupted
checkpoint writes, deployment-before-checkpoint recovery, journal corruption,
wrong firmware, stale/missing records, semantic invalidity, RTC wrap/failure,
power reset, missing task progress, and frozen scheduler time. These tests and
successful builds do not qualify autonomous recovery for flight. Bench-test real
watchdog resets in each phase with outputs isolated, verify RTC and reset-cause
retention through LaunchCore, then test sensor reacquisition and pulse behavior.
A 16-second interruption can be material during flight even when state recovery
works; use measured hardware timing to choose a shorter deadline before flight.
