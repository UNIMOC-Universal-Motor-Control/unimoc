include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.18.0
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

function(add_unimoc_test test_name test_source)
    cmake_parse_arguments(TEST "" "" "LIBRARIES" ${ARGN})

    if(NOT TEST_LIBRARIES)
        message(FATAL_ERROR "No library targets specified for ${test_name}")
    endif()

    add_executable(${test_name} ${test_source})
    target_link_libraries(${test_name} PRIVATE GTest::gtest_main ${TEST_LIBRARIES})
    gtest_discover_tests(${test_name})
endfunction()