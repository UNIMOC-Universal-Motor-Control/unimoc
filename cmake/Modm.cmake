# This file handles the integration of the modm library into the CMake build process.
# It fetches the modm library using FetchContent, sets up a Python virtual environment,
# installs dependencies from requirements.txt, and then uses lbuild to generate
# the necessary build files for modm based on project.xml.
include(FetchContent)

# download the modm library using FetchContent
FetchContent_Declare(
  modm
  GIT_REPOSITORY https://github.com/modm-io/modm.git
  GIT_TAG 2025q1 
)
FetchContent_MakeAvailable(modm)

# check if Python3 is available
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT Python3_Interpreter_FOUND)
    message(FATAL_ERROR "Python3 interpreter not found. Please install Python3.") 
endif()

# set the path to the Python virtual environment
# and the path to the Python binary within that environment
set(VENV_PATH "${CMAKE_SOURCE_DIR}/.venv")
if (WIN32)
  set(VENV_BIN "${VENV_PATH}/Scripts")
else()
  set(VENV_BIN "${VENV_PATH}/bin")
endif()

# Set the path to the requirements.txt file
set(REQUIREMENTS_TXT "${CMAKE_SOURCE_DIR}/requirements.txt")

# Create the Python virtual environment if it does not exist
if(NOT EXISTS "${VENV_PATH}")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m venv "${VENV_PATH}"
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )

    # Activate the virtual environment and install the required packages
    execute_process(
        COMMAND "${VENV_BIN}/python" -m pip install --upgrade pip
    )
    execute_process(
        COMMAND "${VENV_BIN}/pip" install -r "${REQUIREMENTS_TXT}"
    )
endif()

