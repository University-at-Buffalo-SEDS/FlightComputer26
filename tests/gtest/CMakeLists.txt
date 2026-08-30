cmake_minimum_required(VERSION 3.22)
project(flight_computer_host_gtests LANGUAGES C CXX)

enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    GIT_SHALLOW TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

add_library(panic_match_under_test
  "${CMAKE_CURRENT_LIST_DIR}/../../Core/Src/panic_match.c")
target_include_directories(panic_match_under_test PUBLIC
  "${CMAKE_CURRENT_LIST_DIR}/../../Core/Inc")

add_executable(firmware_unit_tests test_panic_match.cpp)
target_link_libraries(firmware_unit_tests PRIVATE panic_match_under_test GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(firmware_unit_tests)
