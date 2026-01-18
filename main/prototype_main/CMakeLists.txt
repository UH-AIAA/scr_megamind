# Cmake header
cmake_minimum_required(VERSION 3.16)

# Project name
project(prototypeMain LANGUAGES CXX)

# Find OpenCppCoverage executable
find_program(OPENCPPCOVERAGE_EXE 
            NAMES OpenCppCoverage.exe OpenCppCoverage)

if(NOT OPENCPPCOVERAGE_EXE)
  message(FATAL_ERROR "OpenCppCoverage not found. Install it or add it to PATH.")
endif()

# Compile logic for testing (library)
add_library(prototypeMain_lib
  src/state_logic.cpp
)

# Add root dependecies
target_include_directories(prototypeMain_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/constants
)

#Add GTest
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../test_components/googletest
  ${CMAKE_BINARY_DIR}/googletest_build
)

# Enable CTest infrastructure to add GTest
enable_testing()

# Test executable
add_executable(prototypeMain_test
  test/test_prototype_main.cpp
)

# Linking test and libraries with GTest functions
target_link_libraries(prototypeMain_test
  PRIVATE
    prototypeMain_lib
    GTest::gtest_main
)

# Include GTest and discover all test case(s)
include(GoogleTest)
gtest_discover_tests(prototypeMain_test)

add_custom_target(coverage
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_BINARY_DIR}/coverage # Remove expired coverage files
  COMMAND "${OPENCPPCOVERAGE_EXE}"                                # Tool path
          --cover_children                                        # Include all child process/thread
          --export_type html:${CMAKE_BINARY_DIR}/coverage         # Generate HTML report inside coverage folder
          -- $<TARGET_FILE:prototypeMain_test>                    # Target test
        DEPENDS prototypeMain_test                                # Ensure test builds before running command
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}                     # Placing log in build folder
)