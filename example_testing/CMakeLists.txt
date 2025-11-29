cmake_minimum_required(VERSION 3.16)

#Name the project: "example"
project(example)
#Add .cpp file(s) into a library target 
add_library(example_lib src/example.cpp)
#Include header files so example_lib (.cpp files) can use
target_include_directories(example_lib PUBLIC include)
#Add the location of GTest for necessary dependencies
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../test_components/googletest
    ${CMAKE_BINARY_DIR}/googletest_build
)
#Enable GTest testing
enable_testing()
#Create an executable to run test name: "example_test"
add_executable(example_test test/test_example.cpp)
#When building the executable example_test, link it with "exampl_lib" and gtest_main 
#PRIVATE: only example_test relies on example_lib and gtest_main
target_link_libraries(example_test 
                     PRIVATE
                     example_lib
                     GTest::gtest_main)
