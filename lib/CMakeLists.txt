# Specify the minimum CMake version
cmake_minimum_required(VERSION 3.31)

# Add the library target
add_library(unimoc_lib)

# Include the headers from the inc subfolder
target_include_directories(unimoc_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/system)

# Add the source files from the src subfolder
target_sources(unimoc_lib
    PRIVATE
    system/SinCos.cpp)

target_link_libraries(unimoc_lib PUBLIC project_options project_warnings units)

