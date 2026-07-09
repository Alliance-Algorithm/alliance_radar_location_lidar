cmake_minimum_required(VERSION 3.24)
project(radar_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# No .msg files yet — add them under msg/ once radar_camera's detection
# contract is finalized, then list them below, e.g.:
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/CameraDetection.msg"
#   DEPENDENCIES std_msgs
# )

ament_package()
