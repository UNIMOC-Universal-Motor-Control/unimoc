# CMakeLists.txt for tests directory
cmake_minimum_required(VERSION 3.20)

# Add test.cpp executable
add_executable(system_test system_test.cpp)
target_link_libraries(system_test gtest gtest_main)
target_link_libraries(system_test unimoc_lib)
# Add test to CMake
add_test(NAME SystemTest COMMAND system_test)