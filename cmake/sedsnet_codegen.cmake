# Match the H523 CPU instead of generic ARMv8-M code generation.
set(SEDSNET_ENV_RUSTFLAGS "-C target-cpu=cortex-m33" CACHE STRING "Rust code generation flags")
if(NOT SEDSNET_ENV_RUSTFLAGS MATCHES "inline-threshold=")
    string(APPEND SEDSNET_ENV_RUSTFLAGS " -Cllvm-args=-inline-threshold=20")
endif()
# Extra ARM machine-outliner iterations miscompile nested helpers in the current
# toolchain: a BL overwrites LR before a tail call to memcpy, looping back into
# that copy with its end pointer as the next length. The C33 simulator captured
# R2=0x20032b60 / LR=0x0806439f and the exact generated helper proves the defect.
# Migrate cached flags too; simply removing the default leaves old builds unsafe.
string(REGEX REPLACE "-C[ ]*llvm-args=-machine-outliner-reruns=[0-9]+" ""
    SEDSNET_ENV_RUSTFLAGS "${SEDSNET_ENV_RUSTFLAGS}")
string(APPEND SEDSNET_ENV_RUSTFLAGS " -Cllvm-args=-machine-outliner-reruns=0")
string(STRIP "${SEDSNET_ENV_RUSTFLAGS}" SEDSNET_ENV_RUSTFLAGS)
set(SEDSNET_ENV_RUSTFLAGS "${SEDSNET_ENV_RUSTFLAGS}" CACHE STRING
    "Rust code generation flags (extra machine outlining disabled)" FORCE)
