# make modm available to this hardware type
include(../../cmake/modm.cmake)

add_subdirectory(modm)

add_executable(${CMAKE_PROJECT_NAME})

target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  adc.cpp
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
  modm_arch_options
  modm_options
  modm_warnings
  modm)
# Outputs hex and bin files.
modm_targets_create(${CMAKE_PROJECT_NAME})

