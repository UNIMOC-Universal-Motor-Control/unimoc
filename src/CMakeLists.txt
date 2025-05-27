# example CMakeLists.txt
#
# Link with project_warnings to use warnings defined in cmake/CompilerWarnings.cmake
# or user modm_warnings to use same warnings as defined for modm. (project_warnings are more strict)

# Add source files here
add_executable(${CMAKE_PROJECT_NAME}
  main.cpp)

# Add include files here
target_include_directories(${CMAKE_PROJECT_NAME}
  PRIVATE
  include)

# Extra compiler options for local project.
#target_compile_options(${CMAKE_PROJECT_NAME}
#  PRIVATE
#  -Wpedantic
#  -Wconversion
#)


target_link_libraries(${CMAKE_PROJECT_NAME}
  project_options
  modm_arch_options
  modm_options
  modm_warnings
  modm)

# Outputs hex and bin files.
modm_targets_create(${CMAKE_PROJECT_NAME})