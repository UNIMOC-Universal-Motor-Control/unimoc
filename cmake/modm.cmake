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


# Führe lbuild build um modm zu bauen
add_custom_command(
    OUTPUT ${CMAKE_SOURCE_DIR}/modm/CMakeLists.txt
    COMMAND "${VENV_BIN}/lbuild" build
    DEPENDS ${CMAKE_SOURCE_DIR}/project.xml
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    COMMENT "Running lbuild build because project.xml changed"
)

