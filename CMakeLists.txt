cmake_minimum_required(VERSION 3.20)
include(cmake/ToolChain.cmake)


include(FetchContent)

FetchContent_Declare(
  modm
  GIT_REPOSITORY https://github.com/modm-io/modm.git
  GIT_TAG 2025q1 
)
FetchContent_MakeAvailable(modm)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(VENV_PATH "${CMAKE_BINARY_DIR}/venv")
if (WIN32)
  set(VENV_BIN "${VENV_PATH}/Scripts")
else()
  set(VENV_BIN "${VENV_PATH}/bin")
endif()

set(REQUIREMENTS_TXT "${CMAKE_SOURCE_DIR}/requirements.txt")

# Erstelle das venv, falls es noch nicht existiert
if(NOT EXISTS "${VENV_PATH}")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m venv "${VENV_PATH}"
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
    endif()

# Installiere Pakete im venv
execute_process(
    COMMAND "${VENV_BIN}/python" -m pip install --upgrade pip
)
execute_process(
    COMMAND "${VENV_BIN}/pip" install -r "${REQUIREMENTS_TXT}"
)


# Führe lbuild build nur aus, wenn das modm-Verzeichnis noch nicht gebaut wurde
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/modm")
  execute_process(
    COMMAND "${VENV_BIN}/lbuild" build
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE lbuild_result
  )
  if(NOT lbuild_result EQUAL 0)
    message(FATAL_ERROR "lbuild build failed!")
  endif()
endif()


# Projektname und Version
project(ModmCMakeTest LANGUAGES C CXX)

add_library(project_options INTERFACE)

# Generate compile_commands.json to make it easier to work with clang based tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Colors is unsupported vscode, can also be turend off in cmake/StandardProjectSettings.cmake
target_compile_options(project_options INTERFACE -fdiagnostics-color=never)

add_library(project_warnings INTERFACE)
include(cmake/Cache.cmake)


include(cmake/Doxygen.cmake)
enable_doxygen()


include(cmake/StaticAnalyzers.cmake)

option(ENABLE_PCH "Enable Precompiled Headers" OFF)
if(ENABLE_PCH)
  target_precompile_headers(
    project_options
    INTERFACE
    <vector>  # Insert preferred headers
    <string>
    <map>
    <utility>)
endif()

add_subdirectory(modm)
add_subdirectory(src)
