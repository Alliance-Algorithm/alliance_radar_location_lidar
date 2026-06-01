cmake_minimum_required(VERSION 4.3)
project(radar_localization_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(small_gicp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(radar_localization_lidar_node src/radar_localization_lidar_node.cpp)
target_link_libraries(radar_localization_lidar_node small_gicp::small_gicp)
ament_target_dependencies(radar_localization_lidar_node rclcpp std_msgs sensor_msgs pcl_conversions tf2_geometry_msgs)
install(TARGETS radar_localization_lidar_node DESTINATION lib/${PROJECT_NAME})
ament_package()
