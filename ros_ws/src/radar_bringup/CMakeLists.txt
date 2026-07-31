cmake_minimum_required(VERSION 3.14)
project(radar_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_competition_contract
    test/test_competition_contract.py
  )
  ament_add_pytest_test(test_registration_gate
    test/test_registration_gate.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_competition_launch
    test/test_competition_launch.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_fusion_bridge_csv_replay
    test/test_fusion_bridge_csv_replay.py
    TIMEOUT 60
  )
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/registration_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME registration_gate
)

ament_package()
