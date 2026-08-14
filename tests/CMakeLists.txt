# CMakeLists.txt for tests directory
cmake_minimum_required(VERSION 4.00)

include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

# Add test executable
add_executable(system_test system_test.cpp)

# define the same hardware limits for the test executable as for the main project
target_compile_definitions(system_test PUBLIC
    HARDWARE_MAX_PHASE_CURRENT_A=100.0
    HARDWARE_MAX_MOTOR_CURRENT_A=80.0
    HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A=50.0
    HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A=20.0
)

target_link_libraries(system_test gtest gtest_main)
target_link_libraries(system_test unimoc_lib)
gtest_discover_tests(system_test)

# Add three phase system test executable
add_executable(three_phase_system_test three_phase_system_test.cpp)
target_link_libraries(three_phase_system_test gtest gtest_main)
target_link_libraries(three_phase_system_test unimoc_lib)
gtest_discover_tests(three_phase_system_test)