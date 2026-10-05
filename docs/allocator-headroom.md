# FC networking heap headroom

The compact packet arena and TLSF profile initially left the 48 KiB primary
telemetry pool with about 42–44 KiB occupied and hundreds of admission refusals.
Incoming CAN packets returned SEDS_IO while allocation-failure counters remained
zero. Basic receive work needs its estimated transient bytes plus protected
reserve; discovery snapshot work requests 8 KiB plus that reserve.

TLSF now owns a dedicated 16 KiB pool in DMA_NO_CACHE SRAM, and the unused
ThreadX application-pool tail after every task stack has been allocated.
Startup logging can initialize TLSF before task creation completes, so adding
that final tail supports an already initialized allocator. Existing stack
allocations remain owned by ThreadX. Duplicate registration is harmless.

The two SD staging buffers become 24 KiB each in TLSF builds, instead of 32 KiB.
Their combined 16 KiB reduction funds the dedicated network pool. ThreadX builds
retain the original SD buffers. Logging cadence is unchanged; tolerance of slow
SD writes is reduced and needs an SD soak test. No thread stack or allocator
admission reserve is reduced. SEDSnet is unchanged.

Receive-result counters distinguish rejected packets from healthy network
activity and retain a bounded 16-byte prefix of the most recent failed frame.

Regression tests reproduce the captured 101-byte rejection, early allocator
initialization, late addition of the shared tail, preservation of live stacks,
and sufficient reserve for discovery. They also exercise receive error handling.
