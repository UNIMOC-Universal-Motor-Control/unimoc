add_executable(${CMAKE_PROJECT_NAME})


target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  main.cpp)

add_subdirectory(logging)


target_link_libraries(${CMAKE_PROJECT_NAME}
  project_options
  modm_arch_options
  modm_options
  modm_warnings
  modm)
# Outputs hex and bin files.
modm_targets_create(${CMAKE_PROJECT_NAME})


