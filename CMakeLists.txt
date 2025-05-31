cmake_minimum_required(VERSION 3.20)

# Projektname und Version
project(unimoc LANGUAGES C CXX)

add_library(project_options INTERFACE)

# Generate compile_commands.json to make it easier to work with clang based tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Colors is unsupported vscode, can also be turend off in cmake/StandardProjectSettings.cmake
target_compile_options(project_options INTERFACE -fdiagnostics-color=never)

add_library(project_warnings INTERFACE)
include(cmake/Cache.cmake)

# recomended warniings for modm projects
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

include(cmake/StaticAnalyzers.cmake)

# add universal code as library
add_subdirectory(lib)

option(ENABLE_TESTS "Enable tests" OFF)

if(ENABLE_TESTS)
  add_subdirectory(tests)
else()
  # generate docs only for the main project
  include(cmake/Doxygen.cmake)
  enable_doxygen()
  # use modm only if embedded target
  include(cmake/modm.cmake)
  add_subdirectory(modm)

  # add the projects code folders
  add_subdirectory(src)
endif()



