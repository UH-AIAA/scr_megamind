cmake_minimum_required(VERSION 3.16)

project(prototypeMain LANGUAGES CXX)

# Library: compile logic for testing
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

enable_testing()

# Test executable
add_executable(prototypeMain_test
  test/test_prototype_main.cpp
)

target_link_libraries(prototypeMain_test
  PRIVATE
    prototypeMain_lib
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(prototypeMain_test)
