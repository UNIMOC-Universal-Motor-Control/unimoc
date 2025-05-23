# Specify the minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Add the library target
add_library(unimoc_lib)

# use unit library
target_compile_definitions(unimoc_lib PRIVATE UNIT_LIB_DISABLE_IOSTREAM)
target_compile_definitions(unimoc_lib PRIVATE DISABLE_PREDEFINED_UNITS)
target_compile_definitions(unimoc_lib PRIVATE ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS)
target_compile_definitions(unimoc_lib PRIVATE ENABLE_PREDEFINED_TORQUE_UNITS)
target_compile_definitions(unimoc_lib PRIVATE ENABLE_PREDEFINED_MAGNETIC_FLUX_UNITS)
target_compile_definitions(unimoc_lib PRIVATE ENABLE_PREDEFINED_ENERGY_UNITS)
target_include_directories(unimoc_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/units)

# Include the headers from the inc subfolder
target_include_directories(unimoc_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/system)

# Add the source files from the src subfolder
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
target_sources(unimoc_lib PRIVATE ${SRC_FILES})