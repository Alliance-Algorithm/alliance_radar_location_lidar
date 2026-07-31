cmake_minimum_required(VERSION 3.24)
project(radar_interfaces)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GameState.msg"
  "msg/LidarLocation.msg"
  "msg/CameraDetectionPose.msg"
  "msg/RegistrationStatus.msg"
  "msg/CameraDetection.msg"
  "msg/CameraDetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  ament_add_pytest_test(test_camera_detection_interfaces
    test/test_camera_detection_interfaces.py
  )
endif()

ament_package()
