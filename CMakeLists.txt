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
    ${CMAKE_SOURCE_DIR}/Core/Src/dma.c
    ${CMAKE_SOURCE_DIR}/Core/Src/barometer.c
    ${CMAKE_SOURCE_DIR}/Core/Src/gyroscope.c
    ${CMAKE_SOURCE_DIR}/Core/Src/accelerometer.c
    ${CMAKE_SOURCE_DIR}/Core/Src/evaluation.c
    ${CMAKE_SOURCE_DIR}/Core/Src/kalman.c
    ${CMAKE_SOURCE_DIR}/Core/Src/distribution.c
    ${CMAKE_SOURCE_DIR}/Core/Src/recovery.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sd_card.c 
    ${CMAKE_SOURCE_DIR}/Core/Src/can_bus.c 

    # ARM matrix functions
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_cholesky_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/DSP/Include
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
option(ENABLE_TELEMETRY "Enable the telemetry subsystem" ON)

# Print its state for debugging
message(STATUS "Telemetry enabled: ${ENABLE_TELEMETRY}")

if (ENABLE_TELEMETRY)
    # Define or undefine the macro globally
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TELEMETRY_ENABLED)

    set(SEDSPRINTF_RS_DEVICE_IDENTIFIER "FC" CACHE STRING "" FORCE)
    set(SEDSPRINTF_RS_MAX_STACK_PAYLOAD "128" CACHE STRING "" FORCE)
    set(SEDSPRINTF_RS_ENV_MAX_QUEUE_SIZE "4096" CACHE STRING "" FORCE)
    set(SEDSPRINTF_RS_ENV_STARTING_QUEUE_SIZE "512" CACHE STRING "" FORCE)
    set(SEDSPRINTF_RS_ENV_QUEUE_GROW_STEP "1.25" CACHE STRING "" FORCE)
    set(SEDSPRINTF_RS_ENV_MAX_RECENT_RX_IDS "32" CACHE STRING "" FORCE)
    set(SEDSPRINTF_RS_ENV_STARTING_RECENT_RX_IDS "16" CACHE STRING "" FORCE)
    set(SEDSPRINTF_RS_ENV_PAYLOAD_COMPRESS_THRESHOLD "30" CACHE STRING "" FORCE)

    # Set target for embedded systems
    set(SEDSPRINTF_RS_TARGET "thumbv8m.main-none-eabihf" CACHE STRING "" FORCE)

    target_sources(${PROJECT_NAME} PRIVATE
        # Telemetry sources (FC-specific)

        ${CMAKE_SOURCE_DIR}/Core/Src/telemetry.c
        ${CMAKE_SOURCE_DIR}/Core/Src/telemetry_thread.c
        ${CMAKE_SOURCE_DIR}/Core/Src/telemetry_hooks.c
    )

    add_subdirectory(${CMAKE_SOURCE_DIR}/sedsprintf_rs)
    target_link_libraries(${CMAKE_PROJECT_NAME} sedsprintf_rs)
endif()


# Packet handling options

# Option defaults to OFF
option(MESSAGE_BATCHING "Handle batched messages in FC handler." OFF)
message(STATUS "Message batching: ${MESSAGE_BATCHING}")

if (MESSAGE_BATCHING)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE MESSAGE_BATCHING_ENABLED)
endif()

# Option defaults to ON
option(TELEMETRY_COMPAT "FC command compatibility with 1-byte payload." ON)
message(STATUS "Command compatibility: ${TELEMETRY_COMPAT}")

if (TELEMETRY_COMPAT)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE TELEMETRY_CMD_COMPAT)
endif()

# Option defaults to ON
option(EXTERNAL_GPS "Availability of external GPS module." ON)
message(STATUS "External GPS: ${EXTERNAL_GPS}")

if (EXTERNAL_GPS)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE GPS_AVAILABLE)
endif()

# Option defaults to ON
option(ONBOARD_SD "Logger storage on FC board." ON)
message(STATUS "SD card: ${ONBOARD_SD}")

if (ONBOARD_SD)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SD_AVAILABLE)
endif()


# DMA testing

# Option defaults to OFF
option(DMA_TESTING "Local DMA testing (outputs to terminal emulator)" OFF)

if (DMA_TESTING)
    message(STATUS "Running local DMA tests (no telemetry).")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE DMA_LOCAL_TEST)
endif()

# Options defaults to OFF
option(DMA_BENCH "DMA produce/consume delay benchmark" OFF)

if (DMA_BENCH)
    message(STATUS "Running DMA benchmarks.")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE DMA_BENCHMARK)
endif()


# Custom compilation and linker flags

# Option defaults to OFF
option(CUSTOM_FLAGS "User-defined flags" OFF)
message(STATUS "Custom flags: ${CUSTOM_FLAGS}")

if (CUSTOM_FLAGS)
    set(FC_COMMON   "-mcpu=cortex-m33" "-mfpu=fpv4-sp-d16" "-mfloat-abi=hard")

    set(COMP_DEBUG    "-Og" "-g")
    set(COMP_RELEASE  "-O3" "-flto" "-ffast-math" "-funroll-loops" "-DNDEBUG" "-g0")

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

    # Newlib is compiled without LTO, so we must maintain this for syscall
    # wrappers that CubeMX generated.
    set_source_files_properties(${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
                                PROPERTIES COMPILE_OPTIONS "-fno-lto")
endif()