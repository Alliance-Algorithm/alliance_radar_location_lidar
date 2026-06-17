cmake_minimum_required(VERSION 3.14)
project(radar_fusion)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG -DEIGEN_NO_DEBUG")
add_compile_options(-flto)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

file(GLOB SOURCES "src/*.cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(radar_fusion_node ${SOURCES})
ament_target_dependencies(radar_fusion_node
    rclcpp
    geometry_msgs
    tf2_geometry_msgs
)

install(TARGETS radar_fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION ${PROJECT_NAME})
ament_package()
