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

#Telemetry library

set(SEDSPRINTF_RS_DEVICE_IDENTIFIER "FC" CACHE STRING "" FORCE)

set(SEDSPRINTF_RS_TARGET "thumbv8m.main-none-eabihf" CACHE STRING "" FORCE) # Set target for embedded systems
add_subdirectory(${CMAKE_SOURCE_DIR}/sedsprintf_rs)
target_link_libraries(${CMAKE_PROJECT_NAME} sedsprintf_rs)

# Option defaults to ON
option(ENABLE_TELEMETRY "Enable the telemetry subsystem" ON)

# Print its state for debugging
message(STATUS "Telemetry enabled: ${ENABLE_TELEMETRY}")

# Define or undefine the macro globally
if(ENABLE_TELEMETRY)
    add_compile_definitions(TELEMETRY_ENABLED)
else()
    # Not defined → disabled
endif()

# Testing

# Option defaults to OFF
option(TEST_ON_HOST "Build for testing on host machine" OFF)

if(TEST_ON_HOST)
    message(STATUS "Building test target for host")
    enable_language(C)
    include(CTest)
    set(FC_TEST fctest)
    set(HOST_TEST_SOURCES
        tests/emu-core.c
        tests/emu-drivers.c
        tests/emu-sensors.c
    )
    set(ENABLE_TELEMETRY OFF CACHE BOOL "" FORCE)
    add_executable(fctest ${HOST_TEST_SOURCES} tests/run-tests.c)
    target_link_libraries(fctest PRIVATE m pthread)
    target_include_directories(fctest PRIVATE
        ${CMAKE_SOURCE_DIR}/tests
        ${CMAKE_SOURCE_DIR}/Core/Inc
    )
    add_test(NAME ${FC_TEST} COMMAND
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/fctest)
    target_compile_definitions(fctest PRIVATE PL_HOST)
endif()