# FC operator Stop and Startup recovery

The dedicated `BOARD_RECOVERY_FC` endpoint supports explicit query and Startup-only reset. It does not implement Idle or resync. Queries report the actual evaluator phase, boot generation, recovery epoch, and commanded pyro GPIO readback. GPIO registers do not prove a parachute or charge's physical condition.

An explicit global manual Stop uses the existing ABORT type with exactly the 21-byte payload `SEDS_MANUAL_ABORT_V1:`. FC accepts that marker only from the logical GS/RRLS owner or its resolved address. Legacy/unmarked and automatic ABORT payloads do not acquire a new FC effect. Legacy FlightCommand 16 remains an evaluator-only stop; it is not the new global physical Stop.

Manual Stop is not gated by ordinary recovery UTC/floor checks. It first seals the retained watchdog domain, then under the same IRQ guard inhibits every new pulse, retires both pulse deadlines, and writes both pyro outputs LOW. SDK receipt acknowledges admission; actual query readback is required for confirmation. A delayed marked Stop remains a conservative Stop: supervisor/transport retirement is a separate contract, not a timestamp-floor guarantee.

Startup reset first verifies the retained Stop seal by volatile readback before any replay-floor or state-cache write. Barrier failure reports IO and performs no persistent reset work; runtime outputs remain inhibited and LOW. Startup reset uses one bounded asynchronous mailbox. The evaluator must reach a lock-free safe point; the control owner never terminates a lock-owning task. Persistent Startup write/readback and SDK cache retirement run on the telemetry owner. A new Stop epoch cancels completion, including during persistent I/O. Terminal IO/stale outcomes are retained for the exact operator nonce and body: a new SDK packet ID cannot silently retry persistent work. Pending/OK duplicates may observe their existing owner; repair after failure requires a new validated operator nonce. A pending different owner cannot be adopted. Acquisition, clocks, calibration, log history and recording shutdown state remain intact. Late Postinit commits and ignition retries are fenced by the current epoch.

Startup success does not rearm deployment. The retained Stop seal and new-flight hold persist until a subsequent authorized Launch after completed Postinit/Armed preparation. That new mission advances the epoch and appends a newer zero current-flight action mask, retaining the other bounded historical journal slot and all SD history. This seal is retained across watchdog resets; ordinary power-on/brownout initialization retains the existing cold-boot behavior. Old checkpoint seals are invalidated; malformed/unknown retained seals, bad journal CRC/image or clock failures stay fail-closed. Ordinary autonomous sensor recovery remains available outside an explicit Stop/reset hold.

Recovery source time must exceed every verified/attempted canonical timestamp across all issuers. Changing issuer cannot bypass UTC rollback; recovery stays blocked until time passes the floor. This does not disable the manual LOW-only Stop path.

Already SDK-accepted outbound ignition cannot be unsent by an FC local reset. The downstream board's Stop/floor policy and supervisor ordering remain separate from local queued-decree retirement.

## Embedded SDK artifact requirement

FC links the SDK C static library and requests `--crate-type staticlib` through
`SEDSNET_CRATE_TYPE`. This needs the SEDSnet 4.3.6 output selector (or an explicitly
reviewed compatible source). Public 4.3.5's combined rlib/staticlib build path is
rejected at configure time. Rust SDK consumers retain their existing default
artifacts; FC does not patch an external cached SDK or duplicate its Cargo build
orchestration.

The SDK's existing `release-embedded` profile supplies `opt-level = "z"`, fat LTO,
and one codegen unit. FC adds no redundant LTO flags. Acquisition, queues, stack
allocations, logging precision, and the original full-cache retirement behavior
are unchanged. Build and MAIN-slot headroom must be measured on the final source;
prototype fit does not qualify a different source or hardware deployment.
