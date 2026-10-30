# Flight computer SD recordings

The SD logger uses the DAQ CSV header and numeric formatting. Each consumed
barometer, accelerometer and gyroscope acquisition produces a raw/converted row;
network report-rate changes do not decimate this stream. Each completed Kalman
output produces an `EKF` or `DKF` row independently of network reporting. Existing
summary and diagnostic records remain in the same file.

Raw rows are `barometer_sample`, `gyroscope_sample`, and `accelerometer_sample`.
Vectors in `raw_adc_code` and `raw_value` contain sensor register codes;
`value` and `calibrated_value` contain their converted values. Barometer vector
order is pressure, temperature, altitude. Its third raw code is `nan`: there is
no altitude ADC register. Gyroscope/accelerometer vectors are X, Y, Z. Raw float
formatting preserves all signed16 and unsigned24 sensor codes. EKF order is
quaternion (four components), altitude, velocity; DKF order is longitude,
latitude, altitude, velocity.

`monotonic_ms` and `uptime_ticks` are the independent extended millisecond counter;
`uptime_frequency_hz` is 1000. `timestamp_ms` uses a cached network UTC mapping when
valid and monotonic time otherwise, as on DAQ. `time_source` explicitly distinguishes
`network` from `local`. The writer refreshes the mapping once per second; a
mapping older than two seconds, an event predating that mapping, or unavailable
UTC yields local time. Producers never acquire the router mutex. These are
consumption/completion timestamps, not new hardware sample-edge measurements.
Network clock corrections can move UTC backwards; uptime remains ordered.

`pyro_gpio_high` records that software asserted the drogue or reef output after
the existing interlocks and persistent deployment guard accepted it.
`pyro_gpio_low` records the timed deassertion. `pyro_rejected` and
`pyro_already_active` distinguish refused/repeated requests. These records do
not prove igniter current, physical chute deployment or parachute inflation.
The SysTick pulse shutoff remains independent of the SD task, network and FileX.
Capture only copies a timestamp and event into a 32-entry fixed FIFO under short
interrupt exclusion; it performs no formatting, allocation or I/O. Failed SD
append retains the event head for retry. `g_sd_event_pending`,
`g_sd_event_queue_peak` and `g_sd_event_queue_drops` expose pressure.

An accepted FC launch command records `launch_accepted`; detected boost advancing
the local state to `Launch` records `launch_detected`. Either switches subsequent file writes to `<prefix>_LAUNCH-<time>-<suffix>.csv`.
Prelaunch files keep the ordinary prefix. A recovered post-Armed flight also
uses the launch prefix on startup. Rotation flushes/closes the old file only
after its partial-write retry completes; pending rows are retained if flush,
close, create, open or header write fails. A new launch file can include the
buffered prelaunch tail; use its timestamped launch row to locate the boundary.
Files continue rotating below 3 GiB and remain launch-labelled. Creation checks
up to 1000 collision suffixes; existing files are never opened for overwrite.
Before UTC is known the filename uses local uptime. A reset starts another
uniquely created file rather than truncating the old recording.

The authoritative transition to local `Landed` starts a 20-second monotonic
window. Repeated Landed notifications, the following Recovery state and UTC
corrections do not extend it. Raw samples, Kalman outputs and pyro audit records
continue through this window. At its deadline the append path marks a bounded
buffer boundary: subsequent rows, including later pyro audit events, go to a
new ordinary file. A queued pyro event serviced after this boundary keeps its
original capture timestamp in the ordinary audit file; the closed launch file
is not reopened for delayed events. The worker completes retained launch rows, flushes and closes
the launch file before writing the ordinary suffix. A partial write or storage
failure retains both portions and retries the transition; physical I/O may finish
later than the deadline, but no newly appended post-deadline data enters the
launch file. `g_sd_launch_closed` counts completed transitions. An idle launch
file closes even if no later sensor row arrives. Acquisition, ordinary SD
logging and network reporting continue. A later accepted/detected launch after
rearming creates another unique launch file; a stale launch event while Landed
cannot restart the launch recording. Boot/recovery while already Landed starts
a bounded grace window using that boot's monotonic clock and a new unique file.

Both 32/16 KiB buffer configurations remain bounded. Initialized buffers accept
records while an existing card opens. Full buffers and lock contention reject
records immediately, leaving acquisition/control running. Partial FileX writes
retain the original offset and buffer ownership, so retries overwrite the
partial prefix instead of duplicating it. Short tails are handed to the writer
at the one-second media flush deadline even if no full buffer is produced.
Clean shutdown drains both raw tails, then queued audit events with bounded
retries and accounts for records/events it cannot save. Existing deployment
interlocks, pulse duration, sensor and radio rates are unchanged.

Inspect `g_sd_enqueued_records`, `g_sd_written_records`, `g_sd_flushed_records`,
`g_sd_write_errors`, `g_sd_format_errors`, `g_sd_lock_contention_drops`,
`g_sd_unavailable_drops`, `g_sd_shutdown_lost_records`, and
`g_sd_shutdown_lost_events`. A full event FIFO explicitly counts rejected events.
CSV sequence numbers continue across parts and rejected formatting attempts;
producer rejects can therefore leave gaps.

This is bounded stall absorption, not a guarantee of zero loss. At full raw and
filter cadence, the existing buffers may cover less than one second of card
stall; the actual tolerance depends on sample rate and CSV row length. Abrupt
power removal can lose queued rows, unflushed filesystem metadata and card
internal buffers. No physical-card sustained-rate or power-cut qualification is
claimed by host tests or an ARM build. FileX fault-tolerant support, both FAT
copies, bounded nonblocking admission and partial-write safeguards remain enabled.
