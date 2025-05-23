cmake_minimum_required(VERSION 4.0)
project(unimoc_lib LANGUAGES CXX)

# Set C++ standard to C++23
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Fetch GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
FetchContent_MakeAvailable(googletest)

# Include the lib folder
add_subdirectory(${CMAKE_SOURCE_DIR}/lib)

# Enable testing
enable_testing()

# # Add tests (example)
# add_executable(example_test tests/example_test.cpp)
# target_link_libraries(example_test gtest gtest_main)
# target_link_libraries(example_test unimoc_lib)
# # Add test to CMake
# add_test(NAME example_test COMMAND example_test)