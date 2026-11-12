# Generic ARMv8-M Mainline code generation (the Rust target's default CPU).
# Naming a CPU here (e.g. cortex-m33) gives SEDSnet functions target attributes
# the precompiled core library lacks, so LTO cannot inline the panic handler,
# which ignores its message, into core's panic paths. Every panic site then
# keeps its message, location and the core::fmt code: about 17 KB of flash.
# The explicit inline threshold (20) also cost about 1 KB against opt-level z.
# Migrate cached flags too, keeping any other flags a developer supplied.
set(SEDSNET_ENV_RUSTFLAGS "" CACHE STRING "Rust code generation flags")
string(REGEX REPLACE "-C[ ]*target-cpu=[^ ]+" ""
    SEDSNET_ENV_RUSTFLAGS "${SEDSNET_ENV_RUSTFLAGS}")
string(REGEX REPLACE "-C[ ]*llvm-args=-inline-threshold=[0-9]+" ""
    SEDSNET_ENV_RUSTFLAGS "${SEDSNET_ENV_RUSTFLAGS}")
# Extra ARM machine-outliner iterations miscompile nested helpers in the current
# toolchain: a BL overwrites LR before a tail call to memcpy, looping back into
# that copy with its end pointer as the next length. The C33 simulator captured
# R2=0x20032b60 / LR=0x0806439f and the exact generated helper proves the defect.
# Migrate cached flags too; simply removing the default leaves old builds unsafe.
string(REGEX REPLACE "-C[ ]*llvm-args=-machine-outliner-reruns=[0-9]+" ""
    SEDSNET_ENV_RUSTFLAGS "${SEDSNET_ENV_RUSTFLAGS}")
string(APPEND SEDSNET_ENV_RUSTFLAGS " -Cllvm-args=-machine-outliner-reruns=0")
string(REGEX REPLACE "  +" " " SEDSNET_ENV_RUSTFLAGS "${SEDSNET_ENV_RUSTFLAGS}")
string(STRIP "${SEDSNET_ENV_RUSTFLAGS}" SEDSNET_ENV_RUSTFLAGS)
set(SEDSNET_ENV_RUSTFLAGS "${SEDSNET_ENV_RUSTFLAGS}" CACHE STRING
    "Rust code generation flags (extra machine outlining disabled)" FORCE)
