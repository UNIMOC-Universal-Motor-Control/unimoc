# Specify the minimum CMake version
cmake_minimum_required(VERSION 4.00)

# Add the library target
add_library(unimoc_lib INTERFACE)

# Include the headers from the inc subfolder
target_include_directories(unimoc_lib INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/system
    ${CMAKE_CURRENT_SOURCE_DIR}/units
    ${CMAKE_CURRENT_SOURCE_DIR}/control
    ${CMAKE_CURRENT_SOURCE_DIR}/observer)

target_link_libraries(unimoc_lib INTERFACE project_options project_warnings)


add_subdirectory(cymon)
