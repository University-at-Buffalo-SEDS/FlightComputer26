# Flight computer task layout

This page maps the flight computer's (FC) threads, interrupts and the
objects between them as of `origin/dev` 9a3eade, lists the parts that are
harder than they need to be, and proposes a layout that follows the other
boards (ValveBoard26, ActuatorBoard26, DAQ-Board, PowerBoard26, RFBoard26,
gateway-board26). The last section is the migration plan: which steps are
done, which are waiting, and which files other work is changing.

`docs/data-pipeline.md` section 3 is the reference for rates and buffers;
this page is about who runs what.

## 1. Current layout

ThreadX ticks at 1 kHz. Lower number = higher priority.
`TX_DISABLE_PREEMPTION_THRESHOLD` and `TX_TIMER_PROCESS_IN_ISR` are defined
(`tx_user.h`), so any preemption threshold other than the thread's own
priority becomes 0, and ThreadX timers run inside the SysTick interrupt.

### 1.1 Threads

| Thread (`TX_THREAD`) | File | Prio | Threshold | Slice | Stack | Trigger | Work |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Recovery (`recovery_task`) | `recovery.c` | 0 | 0 | none | 12 KiB | `seds_syscall` queue (8 x 1 ULONG), 250 ms timeout | Flight commands, launch/postinit admission, sensor (re)init, manual Deploy/Expand (including the filter swap), config options, recovery-owner service (Stop, reset, sensor restart), bench monitor poll. Sleeps 1 tick at start so telemetry can build the router first |
| Telemetry (`telemetry_task`) | `telemetry.c` | 5 | 5 | 50 | 48 KiB | CAN RX ISR / `telemetry_wake()` event flags, else 1 tick | CAN RX, SEDSnet router queues, discovery, `BOARD_HEALTH`, time sync, flight-state republish (2 s), Abort status push. Runs every packet handler, including `on_fc_packet` (`distribution.c`) that turns network commands into `seds_syscall` messages and stores GPS fixes |
| DMA (`dma_task`) | `dma.c` | 5 | 5 | none | 4 KiB | `dma_ready` event flags (DRDY ISR), 1-tick poll | Picks the next sensor with a QMC-minimised 6-input boolean selector, starts one 8-byte SPI1 DMA read, waits on `dma_completion`, copies the bytes into `taskrx[]` under a spinlock, sets `relv` and `sample_ready` |
| Distribution (`distribution_task`) | `distribution.c` | 5 | 5 | 25 | 16 KiB | `sample_ready` event flags, 1-tick poll | Converts samples (`try_fetch_*` in `dma.c`: compensation, baro ground reference, SD raw rows, IGT override), validates them, publishes IMU/BARO to SD and network, GPS rail tracking, the pad loops (streaming, 5 s postinit averaging), the ascent half of the estimator pipeline (stages IMU then baro for the evaluator) and, after Apogee, the whole descent KF **and** the flight state machine |
| Evaluation (`evaluation_task`) | `evaluation.c` | 5 | **0** | 25 (ignored with a threshold) | 12 KiB | `eval_stage` event flags; created suspended, resumed at Launch | Ascent EKF predict (IMU stage), update + flight state machine (baro stage), from Launch to Apogee. Idle afterwards (the IMU is disabled, so its stage never completes) |
| SD (`fx_app_thread`) | `app_filex.c` → `sdpipeline.c` | 5 | 5 | none | 8184 B | `line.full` semaphore, 50 ms | Mount (retried on `SD_Retry`), SD event FIFO, buffer hand-off, FileX writes, 1 s flush, IGT CSV reads, SD read service between writer passes and after the writer stops |
| (timer) `monotonic_checks` | `recovery.c` | SysTick ISR | – | – | – | 900 ms, then every 450 ms | GS heartbeat loss (3 s), GPS gap (200 ms) → `seds_syscall` |

Stacks total 100,344 B out of the 98,304 B ThreadX application pool plus
the FileX pool (`app_azure_rtos_config.h`); the pool tail becomes the TLSF
fallback region (`telemetry_tlsf_register_pool`).

### 1.2 Objects between threads

| Object | Kind | Producers → consumer | Purpose |
| --- | --- | --- | --- |
| `seds_syscall` | `TX_QUEUE`, 8 x 1 ULONG | telemetry (`on_fc_packet`), distribution (sensor/GPS reports, Postinit → `Reinit_Sensors`), evaluation (`kalman.c` `Disable_IMU`, `Reinit_*`), allocator hooks (`fchooks.c`), timer ISR → recovery | Every command and internal report. Internal vs network is a bit (`fc_mask`) in the same u32; the network origin is recovered by comparing with the global `g_last_network_flight_command_msg` |
| `g_conf` | one `atomic_uint_fast32_t` | everyone | Operator options **and** inter-thread handshakes: `Ascent_KF_Staged`, `Using_Ascent_KF`, `Postinit_Requested`, `Launch_Requested`, `SD_Pipeline_Reset`, `Eval_Abort_Flag`, `CO2_Asserted`, ... |
| `eval_stage` | event flags | distribution → evaluation (recovery sets all bits to wake it) | "an IMU pair / a baro sample is in `meas`" |
| `Ascent_KF_Staged` + local `EVALUATION_STAGED` | `g_conf` bit + bit 7 of a local | evaluation ↔ distribution | two-stage predict/update hand-shake: distribution offers the IMU while the bit is clear and the baro while it is set |
| `meas`, `meas_locks[IMU/Baro]` | struct + `fc_lock` spinlocks | distribution → `kalman.c` (evaluation) | the latest validated sample |
| `taskrx[]`, `dma_locks[]`, `flags.drdy/relv` | buffers + spinlocks + atomics | DRDY ISR / DMA task → distribution | raw bytes per sensor, "ready" and "released, not consumed" bits |
| `dma_ready`, `sample_ready` | event flags | ISR → DMA task; DMA task / `distribution_wake()` → distribution | idle waits |
| `dma_completion` | semaphore | SPI DMA ISR → DMA task | transfer done |
| `rfboard` + `rflock` | struct + spinlock | telemetry (GPS packet) → distribution | latest GPS fix |
| `line` (`sd_meta`) + `line.lock`, `line.full` | double buffer + spinlock + semaphore | every producer → SD thread | CSV rows |
| SD event FIFO | 32-entry ring under PRIMASK | SysTick pyro, recovery, evaluation → SD thread | launch/pyro rows with their tick |
| `telemetry_mu` | `TX_MUTEX` (inherit) | every router caller | SEDSnet router |
| `fc_lock`/`fc_concede` | "wake-yield" spinlock | above | CAS; on contention `tx_thread_relinquish()` until free |
| evaluation threshold 0 | scheduler | evaluation vs recovery | a pass is never interleaved with a manual Deploy/Expand's `descent_initialize` (which rebinds the matrices to the 4x4 set) |
| `fc_watchdog_checkpoint` threshold 0 | scheduler | evaluation/distribution vs recovery | the retained checkpoint write is not interleaved with a control reset |

### 1.3 Interrupts and DMA

| Source | NVIC | Work |
| --- | --- | --- |
| EXTI0/4/7 (gyro/accel/baro DRDY) | 0 | `flags.drdy |= pin`, set `dma_ready` |
| GPDMA1 ch0/ch1 + SPI1 | 0 | raise CS, mark valid, post `dma_completion` |
| FDCAN1 | 0 | wake telemetry (`can_bus_rx_ready_isr`), TX pump |
| SDMMC1 | 0 | FileX driver |
| SysTick | SHPR 0x40 | `fc_runtime_tick`: HAL tick, pyro pulse countdown (`deployment_pulse.c`), ThreadX tick and timers (`monotonic_checks`) |
| TIM6 | 15 | HAL tick before ThreadX starts |

### 1.4 Facts the layout must keep

- **Filter swap.** `descent_initialize` repoints the shared Kalman matrices
  to the 4x4 descent set. An ascent predict/update interrupted by it writes
  6x6 results into 4x4 storage. Today only the evaluation thread's threshold
  0 prevents that, and the GS `EvaluationRelax` command (threshold back to 5)
  removes the protection.
- Priorities 1-4 are unused, so a threshold of 1 would equal no threshold.
- SD logging continues through Landed and Recovery; the SD read service runs
  on the SD thread between writer passes and after the writer stops.
- The barometer ground reference (`baro_track_alt`, drift tracking on the
  pad, frozen at Launch) and the gyro/accel rest detection run in `dma.c`
  `try_fetch_*` on the **real** samples, before the IGT override replaces
  what the estimator sees.
- Pyro pulses are timed in SysTick, not by a thread; a command only has to
  reach `deployment_pulse_start`.
- The IGT replay is paced by real sensor samples: each real baro/IMU sample
  is replaced by the replay value at that time (`igt_override_*`).
- The retained flight checkpoint (`fc_watchdog_checkpoint`) is written once
  per estimator cycle and restored before ThreadX starts.

## 2. What the other boards do

| Board | Threads (priority) | Messaging |
| --- | --- | --- |
| ValveBoard26 | telemetry 5, safety 5, main 6, data acquisition 7, PT SD 8 | `thread_comm.[ch]`: one queue + state mutex, `thread_comm_send/receive`, typed accessors |
| ActuatorBoard26 | telemetry 5, safety 6, main 6 | `thread_comm.[ch]`, host test `tests/test_thread_comm.c` |
| DAQ-Board | acquisition 5, live network 6, analog 6, telemetry 6, SD writer 7 | queues per stream |
| PowerBoard26 | sensor 2, telemetry 3 | latest-value slots |
| RFBoard26 | telemetry 3, GPS 4 | – |
| gateway-board26 | telemetry 5, data acquisition 6 | – |

Common conventions:

- one file per thread named `<name>_thread.c` (`telemetry_thread.c`,
  `data_acq_thread.c`, `safety_thread.c`, `sd_writer_thread.c`), with
  `create_<name>_thread(TX_BYTE_POOL *)` and `<name>_thread_entry(ULONG)`,
  declared in one `XX-Threads.h`;
- no time slicing, no preemption thresholds, no runtime priority changes;
- inter-thread commands go through `thread_comm_send()` /
  `thread_comm_receive()`; shared state has accessors, not raw globals;
- one thread per responsibility; the network thread owns the router;
- storage below everything flight-related.

## 3. Premature optimizations and convolutions

Each item says why removing it is safe.

1. **Evaluation thread non-preemptible (threshold 0), toggled by
   Focus/Relax.** A scheduler property protects a data invariant, and an
   operator command can switch it off. Replace with an explicit estimator
   guard (a mutex held for an estimator pass and by the control thread while
   it executes a command): the same exclusion, visible in the code, and
   `EvaluationRelax` can no longer open the race. Safe: the guard excludes
   exactly what threshold 0 excluded (recovery during a pass; ISRs and the
   equal-priority threads were never involved), and the evaluator gives the
   guard up wherever it sleeps today. Later (step 7) the swap moves into the
   estimator thread itself and needs no lock at all.
2. **Two-thread ascent pipeline.** Distribution stages an IMU pair, the
   evaluator predicts, sets `Ascent_KF_Staged` in the global config word,
   distribution sees the bit and stages a baro sample, the evaluator updates
   and clears the bit (with `distribution_wake()` both ways). One estimator
   thread that waits for "IMU pair, then baro" does the same with no
   handshake bits.
3. **The state machine runs on two threads.** Ascent FSM in evaluation,
   descent FSM (and the descent KF) in distribution, because the IMU stops
   at Apogee and the evaluator's stage can no longer complete. One estimator
   thread for the whole flight removes the hand-over and the starved thread.
4. **Distribution does unrelated jobs.** Sensor conversion and publication,
   GPS rail tracking, pad streaming, postinit averaging, the descent filter
   and FSM. Split into acquisition and estimator.
5. **Separate DMA thread with a boolean-minimised selector.** A 4 KiB thread
   whose only job is to start the next SPI read; the selector is a
   Quine-McCluskey-minimised expression of 6 flags that is unreadable and
   untestable. A plain priority (unconsumed gyro, then accel, then baro)
   inside the acquisition thread reads the same sensors at the same rates
   (each sensor's DRDY period, 10-40 ms, is two orders of magnitude longer
   than an 8-byte transfer).
6. **`fc_lock` wake-yield spinlocks.** A CAS loop that `tx_thread_relinquish()`s
   while contended: correct only while every user has the same priority
   (a higher-priority waiter would spin forever), and it guards 6-24 byte
   copies. A PRIMASK-masked copy (as the SD event FIFO and the deployment
   pulse already use) or a single owner does it with no waiting.
7. **Time slices (25/50 ticks) and `tx_thread_relinquish()` after each
   publication when `Eval_Focused`.** Equal-priority round robin that the
   event-driven waits already make unnecessary.
8. **`PARALLEL_PREDICT_UPDATE`.** An experimental byte pool for the Kalman
   scratch that changes the caller's preemption threshold and retries on
   failure; off in every build and no longer meaningful with
   `TX_DISABLE_PREEMPTION_THRESHOLD`. Dead code.
9. **`MESSAGE_BATCHING_ENABLED`.** Raises the telemetry thread to priority 0
   around a batch; references `telemetry_thread`, which does not exist, so
   the option cannot build. Dead code.
10. **Network origin by global correlation.** The queue word is the command
    only, so `on_fc_packet` writes `g_last_network_flight_command_msg` before
    sending and recovery compares; a command equal to a queued internal one
    is misattributed. A two-word message (command, origin) says it directly.
11. **Health checks in the SysTick ISR.** `monotonic_checks` runs in the
    interrupt (timers process in ISR) and posts to the command queue. A
    deadline checked in the control thread's loop (as VB/AB safety threads
    do) keeps the same 450 ms period without ISR work.
12. **Priority 0 recovery plus a 1-tick start delay** so telemetry builds the
    router first. Ordering by sleep; an explicit start order (create the
    control thread after telemetry is ready, or wait on a ready flag) says it.
13. **Macro indirection.** `MrAnalog(x)` (`while (!(x))`), `WE_ARE_SO_BACK`,
    `IT_IS_NOW_OVER`, `Require()` (hidden `mode` argument and `return`),
    `svec()` over globals, `conditional`, `fc_mask`/`fc_unmask` tagging, and
    `g_conf` mixing operator options with handshakes. Pure renames; done per
    file as the files are touched, not in one sweep.

Kept on purpose: the SysTick pyro pulse, the PRIMASK-guarded SD event FIFO,
the SD double buffer with its single writer, the event-driven idle waits
(WFI idle), and the per-thread watchdog progress bits.

## 4. Target layout

| Thread | File | Prio | Stack | Trigger | Responsibility |
| --- | --- | --- | --- | --- | --- |
| Control | `control_thread.c` (from `recovery.c`) | 4 | 12 KiB | `thread_comm` command queue, 250 ms | commands and their admission, launch/postinit, sensor (re)init, pyro start (`deployment_pulse_start`), config, recovery-owner service; GS heartbeat / GPS gap deadlines (from the ISR timer) |
| Telemetry | `telemetry_thread.c` (thread part of `telemetry.c`) | 5 | 48 KiB | CAN RX / wake flags, 1 tick | router, discovery, health, time sync; packet handlers (`fc_packets.c`, from `distribution.c`) post to `thread_comm` |
| Acquisition | `sensor_thread.c` (from `dma.c` + sensor half of `distribution.c`) | 5 | 16 KiB | DRDY event flags | SPI DMA reads, conversion, baro reference, IGT override, SD raw rows, validation, IMU/BARO publication, GPS rail tracking; posts samples to the estimator |
| Estimator | `estimator_thread.c` (from `evaluation.c` + descent half of `distribution.c`) | 5 | 12 KiB | sample mailbox | postinit attitude, ascent EKF, descent KF, the whole flight state machine, the filter swap (single owner), checkpoint |
| SD writer | `sd_writer_thread.c` (thread part of `sdpipeline.c`, `app_filex.c` entry) | 6 | 8 KiB | `line.full`, 50 ms | unchanged: writer, flush, IGT CSV reads, SD read service |

- Priority bands: 4 control (the old 0; nothing else sits above the
  application group), 5 flight-critical acquisition/estimation/network, 6
  storage. No slices, no thresholds, no runtime priority changes.
- `thread_comm.[ch]`: the command queue (`thread_comm_send`,
  `thread_comm_send_network`, `thread_comm_receive`, `thread_comm_flush`),
  the estimator guard and, in the estimator step, the sample mailbox
  (latest IMU pair / baro / GPS with a sequence number, PRIMASK-copied,
  plus one event flag group to wait on). Replaces `seds_syscall`,
  `g_last_network_flight_command_msg`, `eval_stage`, `sample_ready`,
  `meas_locks`, `dma_locks`, `rflock`, `Ascent_KF_Staged`.
- Declarations in `FC-Threads.h` (replacing `fctasks.h`), one block per
  thread like `VB-Threads.h`.
- No separate safety thread: the FC's only time-critical output (pyro) is
  already SysTick-timed, and the heartbeat/GPS deadlines are 200-3000 ms, so
  they fit the control thread's 250 ms wake; one thread fewer saves a stack.

### 4.1 Timing that must not change

| Requirement | Today | Target |
| --- | --- | --- |
| Estimator cycle | one ascent cycle per baro sample (40 ms), IMU pair predicted just before | same order inside one thread |
| Pyro timing | `deployment_pulse_start` from recovery/distribution/evaluation, SysTick countdown | unchanged (control/estimator call it) |
| Launch detection | FSM on each baro cycle, `LAUNCH_CONFIRM_DELAY` sleep | same FSM, same sleeps (outside the guard) |
| IGT pacing | replay sampled at each real sample's tick | unchanged (`try_fetch_*` moves with acquisition) |
| SD flush | 1 s, 50 ms waits | unchanged |
| Watchdog / checkpoint | progress bits per role; checkpoint per estimator cycle | bits keep their roles (ACQUISITION: sensor, SAFETY: estimator, CONTROL: control, NETWORK: telemetry) |
| Command latency | recovery preempts all but an evaluation pass | control preempts all; waits for the guard only while a pass runs |

### 4.2 Budget

RAM is the tight one (93.7 % flight, 95.0 % ground test). Thread stacks
today: 4 + 16 + 12 + 12 + 48 + 8 KiB. Target: 16 (acquisition) + 12
(estimator) + 12 + 48 + 8 = **-4 KiB** (the DMA thread's stack and TCB). The
thread_comm queue grows from 8 x 4 B to 8 x 8 B (+32 B); a mutex is 52 B.
Flash should shrink (one thread and the dead options gone).

## 5. Migration plan

Each step is one commit, behaviour-preserving, verified with the host suite,
both image builds and the IGT host replay (identical results and per-cycle
traces); the scheduling steps also with the native HITL flight.

| # | Step | Files | Status |
| --- | --- | --- | --- |
| 1 | This document | `docs/task-layout.md` | this commit |
| 2 | `thread_comm` command queue with an explicit origin; host test | new `thread_comm.[ch]`, `recovery.c`, `distribution.c`, `kalman.c`, `fchooks.c`, `fcapi.h`, `fccommon.h` | next |
| 3 | Explicit estimator guard instead of threshold 0; no slice on the evaluator | `thread_comm.c`, `evaluation.c`, `recovery.c`, `fctasks.h`, `test_evaluation_preemption.py`, `data-pipeline.md` 9.5 | next |
| 4 | Remove `PARALLEL_PREDICT_UPDATE` and `MESSAGE_BATCHING` code | `kalman.c`, `evaluation.c`, `distribution.c`, `fccommon.h`, `fcapi.h` (CMake/`build.py` flags after the IGT SD-service work lands) | next |
| 5 | Health deadlines from the SysTick timer into the control loop | `recovery.c` | waiting for confirmation |
| 6 | Acquisition thread: DMA task + sensor half of distribution; plain sensor order; spinlocks → PRIMASK copies | `dma.c`, `distribution.c`, `fctasks.h`, `app_threadx.c`, stack tests | waiting for confirmation |
| 7 | Estimator thread: evaluation + descent; filter swap by request; drop `Ascent_KF_Staged`, `eval_stage` | `evaluation.c`, `distribution.c`, `kalman.c`, `recovery.c`, `igt_replay_host.py` driver | waiting for confirmation |
| 8 | Priority bands (control 4, SD 6), no slices, `FC-Threads.h`, file renames | `fctasks.h`, `app_filex.c`, `telemetry.c`, tests | after the SD/link work lands |
| 9 | Macro clean-up per file; `g_conf` handshake bits into `thread_comm` | as touched | last |

Files other work is changing now (rebase before touching):
`sdpipeline.c`, `ground_test.[ch]`, `board_health.c`, `sd_file_wire.h`,
`CMakeLists.txt`, `build.py` (IGT SD service); `telemetry.c`, `can_bus.c`,
the CAN/radio headers (link headers); SEDSnet configuration and
`CMakeLists.txt` (SEDSnet 7). Steps 2-4 avoid all of them; step 8 touches
`telemetry.c` and `app_filex.c` and waits.
