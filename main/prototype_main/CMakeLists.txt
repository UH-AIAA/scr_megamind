cmake_minimum_required(VERSION 3.16)

#Name the project: "example"
project(PrototypeMain)
#Add .cpp file(s) into a library target 
add_library(PrototypeMain_lib src/prototype_main.cpp)
#Include header files so example_lib (.cpp files) can use
target_include_directories(PrototypeMain_lib PUBLIC include)
#Add the location of GTest for necessary dependencies
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test_components/googletest
    ${CMAKE_BINARY_DIR}/googletest_build
)
#Enable GTest testing
enable_testing()
#Create an executable to run test name: "example_test"
add_executable(PrototypeMain_test test/test_prototype_main.cpp)
#When building the executable example_test, link it with "exampl_lib" and gtest_main 
#PRIVATE: only example_test relies on example_lib and gtest_main
target_link_libraries(PrototypeMain_test 
                     PRIVATE
                     PrototypeMain_lib
                     GTest::gtest_main)
