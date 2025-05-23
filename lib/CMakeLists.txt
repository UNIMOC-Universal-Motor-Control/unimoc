# Specify the minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Add the library target
add_library(unimoc_lib)

# use unit library
target_include_directories(unimoc_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/units)

# Include the headers from the inc subfolder
target_include_directories(unimoc_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)

# Add the source files from the src subfolder
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
target_sources(unimoc_lib PRIVATE ${SRC_FILES})