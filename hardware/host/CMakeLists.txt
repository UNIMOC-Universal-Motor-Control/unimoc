if(NOT ENABLE_TESTS)
  add_executable(${CMAKE_PROJECT_NAME})

  find_package(Threads REQUIRED)

  target_sources(${CMAKE_PROJECT_NAME}
    PRIVATE
    hardware_interface.cpp
  )

  target_include_directories(${CMAKE_PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ../
  )

  target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
    unimoc_lib
    Threads::Threads
  )
endif()
