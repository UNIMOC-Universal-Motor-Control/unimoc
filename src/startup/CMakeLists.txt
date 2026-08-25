
# Add source files here
target_sources(${CMAKE_PROJECT_NAME}
  PRIVATE
  hw_startup.cpp)


# Add include files here
target_include_directories(${CMAKE_PROJECT_NAME}
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR})
