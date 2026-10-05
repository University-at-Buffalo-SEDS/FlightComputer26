include(FetchContent)

set(SEDSNET_SCHEMA_FILE "${CMAKE_SOURCE_DIR}/config/sedsnet.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${SEDSNET_SCHEMA_FILE}")

set(SEDSNET_FORCE_RELEASE ON CACHE BOOL
    "Build SEDSNet in release mode for embedded firmware" FORCE)
set(SEDSNET_EMBEDDED_BUILD ON CACHE BOOL "Build SEDSNet for an embedded target" FORCE)
# Match the H523 CPU instead of generic ARMv8-M code generation.
set(SEDSNET_ENV_RUSTFLAGS "-C target-cpu=cortex-m33" CACHE STRING "Rust code generation flags")
set(SEDSNET_ENABLE_CRYPTOGRAPHY OFF CACHE BOOL
    "Keep the current unencrypted embedded transport" FORCE)

include("${CMAKE_SOURCE_DIR}/cmake/sedsnet_source.cmake")

if(SEDSNET_COMPACT_PACKET_STORE AND NOT EXISTS "${SEDSNET_DIR}/src/packet_store.rs")
    message(FATAL_ERROR "Selected SEDSnet lacks the packet arena; fetch dev or supply a current dev checkout")
endif()

FetchContent_Declare(
    sedsnet
    GIT_REPOSITORY https://github.com/Rylan-Meilutis/SEDSnet.git
    GIT_TAG ${SEDSNET_GIT_REF}
    GIT_SHALLOW FALSE
    PATCH_COMMAND ${CMAKE_COMMAND}
                  -DSEDSNET_SOURCE_DIR=<SOURCE_DIR>
                  -DSEDSNET_SCHEMA_FILE=${SEDSNET_SCHEMA_FILE}
                  -DSEDSNET_CRC32_DIR=${CMAKE_SOURCE_DIR}/third_party/embedded-crc32fast
                  -P ${CMAKE_SOURCE_DIR}/cmake/prepare_sedsnet.cmake
)
FetchContent_MakeAvailable(sedsnet)

# Copy the board schema during configure, before Ninja calculates whether the
# Rust archive is stale. Deleting/touching files from a build-time dependency
# races Ninja's initial dirty check and can remove the archive immediately
# before the firmware link step.
configure_file("${SEDSNET_SCHEMA_FILE}" "${sedsnet_SOURCE_DIR}/telemetry_config.json" COPYONLY)
file(TOUCH "${sedsnet_SOURCE_DIR}/build.rs")
# Resolve the C firmware's memory/ABI helpers with the ARM runtime before
# Rust's static archive can supply its larger portable implementations.
# Root the ABI entry points too: they are first referenced inside that archive,
# after the linker has already scanned libc. Its aligned variants are aliases.
target_link_libraries(${CMAKE_PROJECT_NAME} gcc c sedsnet::sedsnet)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-u,memcpy -Wl,-u,memmove -Wl,-u,memset
    -Wl,-u,__aeabi_memcpy -Wl,-u,__aeabi_memmove
    -Wl,-u,__aeabi_memset -Wl,-u,__aeabi_memclr)
add_dependencies(${CMAKE_PROJECT_NAME} sedsnet_build)
