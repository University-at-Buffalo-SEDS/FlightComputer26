cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FlightComputer26)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T "${CMAKE_SOURCE_DIR}/STM32H523xx_FLASH.ld")

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/barometer.c
    ${CMAKE_SOURCE_DIR}/Core/Src/gyroscope.c
    ${CMAKE_SOURCE_DIR}/Core/Src/accelerometer.c
    ${CMAKE_SOURCE_DIR}/Core/Src/dma.c
    ${CMAKE_SOURCE_DIR}/Core/Src/distribution.c
    ${CMAKE_SOURCE_DIR}/Core/Src/evaluation.c
    ${CMAKE_SOURCE_DIR}/Core/Src/kalman.c
    ${CMAKE_SOURCE_DIR}/Core/Src/recovery.c
    ${CMAKE_SOURCE_DIR}/Core/Src/can_bus.c 
    ${CMAKE_SOURCE_DIR}/Core/Src/fchooks.c

    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_atan2_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_scale_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_vec_mult_f32.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
    ${CMAKE_SOURCE_DIR}/Core/Test
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Telemetry library

# Option defaults to ON
option(ENABLE_TELEMETRY "Enable the telemetry subsystem" OFF)

# Print its state for debugging
message(STATUS "Telemetry enabled:      ${ENABLE_TELEMETRY}")

if (ENABLE_TELEMETRY)
    # Define or undefine the macro globally
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TELEMETRY_ENABLED)

    set(SEDSNET_DEVICE_IDENTIFIER "FC" CACHE STRING "" FORCE)
    set(SEDSNET_MAX_STACK_PAYLOAD "8" CACHE STRING "" FORCE)
    set(SEDSNET_MAX_QUEUE_BUDGET "1024" CACHE STRING "" FORCE)
    set(SEDSNET_MAX_RECENT_RX_IDS "32" CACHE STRING "" FORCE)
    set(SEDSNET_ENV_STARTING_QUEUE_SIZE "512" CACHE STRING "" FORCE)
    set(SEDSNET_ENV_QUEUE_GROW_STEP "1.25" CACHE STRING "" FORCE)
    set(SEDSNET_ENV_PAYLOAD_COMPRESS_THRESHOLD "30" CACHE STRING "" FORCE)
    set(SEDSNET_TARGET "thumbv8m.main-none-eabihf" CACHE STRING "" FORCE)

    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Src/telemetry.c
    )

    include(cmake/sedsnet_fetch.cmake)
endif()


# Flight Computer options

option(TELEMETRY_COMPAT "Byte payload compatibility"    ON)
option(EXTERNAL_GPS     "GPS board availability"        ON)
option(ONBOARD_SD       "On-board logger storage"       ON)
option(CUSTOM_FLAGS     "User-defined flags"            ON)
option(FC_BENCH         "Project-wide benchmark"        OFF)
option(PARALLEL_KF      "Predict does not wait Update"  OFF)
option(MESSAGE_BATCHING "Handle batched messages"       OFF)
option(MANUAL_CONFIRM   "Request confirm critical cmd"  OFF)
option(SENSOR_TESTS     "Synchronous sensor tests"      OFF)
option(USB_ENUM         "Assume USB for IO functions"   OFF)
option(DEBUG_MATH       "Report failures of math fns"   OFF)
option(IGNORE_STATES    "Disable recovery state checks" OFF)
option(SWAP_CONFIG      "Use simulation user config"    OFF)
option(EXPORT_SPINLOCK  "Use spinlock in hooked libs"   OFF)
option(ALLOC_TEST       "Test exported allocator."      OFF)
option(TEST_DESCENT     "Start at Apogee."              OFF)
option(FAKESTATION      "Emulate GS commands."          OFF)

message(STATUS "Command compatibility:  ${TELEMETRY_COMPAT}")
message(STATUS "External GPS:           ${EXTERNAL_GPS}")
message(STATUS "SD card:                ${ONBOARD_SD}")
message(STATUS "Custom flags:           ${CUSTOM_FLAGS}")
message(STATUS "Benchmarks:             ${FC_BENCH}")
message(STATUS "Parallel Kalman:        ${PARALLEL_KF}")
message(STATUS "Message batching:       ${MESSAGE_BATCHING}")
message(STATUS "Manual confirms:        ${MANUAL_CONFIRM}")
message(STATUS "Sensor tests:           ${SENSOR_TESTS}")
message(STATUS "USB enumeration:        ${USB_ENUM}")
message(STATUS "Math debugging:         ${DEBUG_MATH}")
message(STATUS "Ignore states:          ${IGNORE_STATES}")
message(STATUS "Simulation:             ${SWAP_CONFIG}")
message(STATUS "Export spinlock:        ${EXPORT_SPINLOCK}")
message(STATUS "Test allocator:         ${ALLOC_TEST}")
message(STATUS "Start at Apogee:        ${TEST_DESCENT}")
message(STATUS "Emulate GS commands:    ${FAKESTATION}")

if (MESSAGE_BATCHING)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE MESSAGE_BATCHING_ENABLED)
endif()

if (TELEMETRY_COMPAT)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TELEMETRY_CMD_COMPAT)
endif()

if (PARALLEL_KF)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE PARALLEL_PREDICT_UPDATE)
endif()

if (MANUAL_CONFIRM)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USER_CONFIRMATION)
endif()

if (EXTERNAL_GPS)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE GPS_AVAILABLE)
endif()

if (ONBOARD_SD)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SD_AVAILABLE)
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Src/sdpipeline.c
    )
endif()

if (FC_BENCH)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FC_BENCHMARK)
endif()

if (SENSOR_TESTS)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TEST_SENSORS)
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Test/sensors.c
    )
endif()

if (USB_ENUM)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE USB_ENUMERATES)
endif()

if (DEBUG_MATH)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE MATH_FN_DEBUG)
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Test/kalmanpill.c
    )
endif()

if (IGNORE_STATES)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE LUNATIC_STATE)
endif()

if (SWAP_CONFIG)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SIMULATION_CONFIG)
endif()

if (EXPORT_SPINLOCK)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE EXPORT_SPINLOCK)
endif()

if (ALLOC_TEST)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TEST_ALLOC)
endif()

if (TEST_DESCENT)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE DESCENT_TEST)
endif()

if (FAKESTATION)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE FAKESTATION)
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Core/Src/telemetry.c
        ${CMAKE_SOURCE_DIR}/Core/Test/fakestation.c
    )
endif()

if (CUSTOM_FLAGS)
    set(FC_COMMON   "-mcpu=cortex-m33" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

    set(COMP_DEBUG    "-Og" "-g")
    set(COMP_RELEASE  "-Os" "-flto" "-ffast-math" "-DNDEBUG" "-g0")

    set(LINK_DEBUG    )
    set(LINK_RELEASE  "-flto")

    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
        ${FC_COMMON}
        $<$<CONFIG:Debug>:${COMP_DEBUG}>
        $<$<CONFIG:Release>:${COMP_RELEASE}>
    )

    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        ${FC_COMMON}
        $<$<CONFIG:Debug>:${LINK_DEBUG}>
        $<$<CONFIG:Release>:${LINK_RELEASE}>
    )

    # Newlib is compiled without LTO; we must preserve this for syscall
    # wrappers that CubeMX generated.
    set_source_files_properties(${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
                                PROPERTIES COMPILE_OPTIONS "-fno-lto")
endif()

set(LAUNCHCORE_DEVICE_DEFINE STM32H523xx)
set(LAUNCHCORE_CMSIS_DEVICE_DIR "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H5xx/Include")
set(LAUNCHCORE_HAL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Inc")
set(LAUNCHCORE_SYSTEM_SOURCE "${CMAKE_SOURCE_DIR}/Core/Src/system_stm32h5xx.c")
set(LAUNCHCORE_STARTUP_SOURCE "${CMAKE_SOURCE_DIR}/startup_stm32h523xx.s")
set(LAUNCHCORE_FLASH_SOURCES
    "${CMAKE_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash.c"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_flash_ex.c"
    "${CMAKE_SOURCE_DIR}/Drivers/STM32H5xx_HAL_Driver/Src/stm32h5xx_hal_hash.c")
set(LAUNCHCORE_HARDWARE_SHA256_SOURCE
    "${CMAKE_SOURCE_DIR}/Bootloader/sha256_hardware.c")
set(LAUNCHCORE_APP_SLOT_BASE 0x08004000)
set(LAUNCHCORE_APP_VECTOR_TABLE 0x08004100)
set(LAUNCHCORE_METADATA0_BASE 0x08078000)
include(cmake/launchcore_stm32.cmake)
