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

function(add_unimoc_test test_name test_source)
    add_executable(${test_name} ${test_source})

    # Keep the hosted test values aligned with the firmware configuration.
    target_compile_definitions(${test_name} PUBLIC
        HARDWARE_MAX_PHASE_CURRENT_A=100.0
        HARDWARE_MAX_MOTOR_CURRENT_A=80.0
        HARDWARE_MAX_BATTERY_DRIVE_CURRENT_A=50.0
        HARDWARE_MAX_BATTERY_CHARGE_CURRENT_A=20.0
    )

    target_link_libraries(${test_name} PRIVATE gtest gtest_main unimoc_lib)
    gtest_discover_tests(${test_name})
endfunction()

add_unimoc_test(three_phase_system_test three_phase_system_test.cpp)
add_unimoc_test(stator_system_test stator_system_test.cpp)
add_unimoc_test(rotor_system_test rotor_system_test.cpp)
add_unimoc_test(rotor_angle_test rotor_angle_test.cpp)
add_unimoc_test(sin_cos_test sin_cos_test.cpp)

add_unimoc_test(node_identity_test node_identity_test.cpp)
add_unimoc_test(nvm_settings_test nvm_settings_test.cpp)
add_unimoc_test(control_mode_test control_mode_test.cpp)
add_unimoc_test(cyphal_interface_test cyphal_interface_test.cpp)
add_unimoc_test(excitation_controller_test excitation_controller_test.cpp)
add_unimoc_test(position_controller_test position_controller_test.cpp)
add_unimoc_test(excitation_observer_test excitation_observer_test.cpp)
add_unimoc_test(position_tracker_test position_tracker_test.cpp)