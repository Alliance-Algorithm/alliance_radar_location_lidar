cmake_minimum_required(VERSION 3.14)
project(radar_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_competition_contract
    test/test_competition_contract.py
  )
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
