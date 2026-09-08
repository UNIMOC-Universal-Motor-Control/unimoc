target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  main.cpp)

add_subdirectory(current_control)
add_subdirectory(startup)
