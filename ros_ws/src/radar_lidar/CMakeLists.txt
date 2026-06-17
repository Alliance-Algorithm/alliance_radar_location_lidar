cmake_minimum_required(VERSION 4.0)
project(radar_lidar)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O3 -march=native -DNDEBUG -DEIGEN_NO_DEBUG -g -DRADAR_DEBUG")
add_compile_options(-flto)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB SOURCES "src/*.cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(radar_lidar_node ${SOURCES})
target_link_libraries(radar_lidar_node
    Eigen3::Eigen
)
ament_target_dependencies(radar_lidar_node
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
    tf2_geometry_msgs
    small_gicp
    Eigen3
)

install(TARGETS radar_lidar_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION ${PROJECT_NAME})
ament_package()
