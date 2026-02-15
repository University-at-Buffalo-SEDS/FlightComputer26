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

# DMA testing

# Options defaults to OFF
option(DMA_TESTING "Local DMA testing (outputs to terminal emulator)" OFF)

if (DMA_TESTING)
    message(STATUS "Running local DMA tests (no telemetry).")
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE DMA_LOCAL_TEST)
endif()