cmake_minimum_required(VERSION 3.14)
project(radar_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(
    NAME competition_launch_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_competition_launch_contract.py
  )
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
