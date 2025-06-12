# Specify the minimum CMake version
cmake_minimum_required(VERSION 4.00)

# Setup the unit library
option(ENABLE_TESTS "Enable tests" OFF)
option(UNIT_LIB_DISABLE_IOSTREAM "Disable iostream in units library" ON)
option(DISABLE_PREDEFINED_UNITS "Disable predefined units in units library" ON)
option(ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS "Enable predefined angular velocity units in units library" ON)
option(ENABLE_PREDEFINED_TORQUE_UNITS "Enable predefined torque units in units library" ON)
option(ENABLE_PREDEFINED_MAGNETIC_FLUX_UNITS "Enable predefined magnetic flux units in units library" ON)
option(ENABLE_PREDEFINED_ENERGY_UNITS "Enable predefined energy units in units library" ON)

# include the units library
add_subdirectory(3rdparty/units)
# fix the data type for the units library to float
target_compile_definitions(units INTERFACE UNIT_LIB_DEFAULT_TYPE=float)
target_compile_options(units INTERFACE $<$<COMPILE_LANGUAGE:CXX>:-fsingle-precision-constant> $<$<COMPILE_LANGUAGE:C>:-fsingle-precision-constant>)

# Add the library target
add_library(unimoc_lib)

# Include the headers from the inc subfolder
target_include_directories(unimoc_lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/system)

# Add the source files from the src subfolder
target_sources(unimoc_lib PRIVATE src/dummy.cpp) 

target_link_libraries(unimoc_lib PUBLIC project_options project_warnings units)

