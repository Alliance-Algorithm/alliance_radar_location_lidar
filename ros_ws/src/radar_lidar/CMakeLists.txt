cmake_minimum_required(VERSION 3.24)
project(radar_lidar)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(small_gicp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search kdtree)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

# ── Core library (runtime algorithm; ROS-free) ─────────────────────
add_library(${PROJECT_NAME}_core STATIC
    src/map_data.cpp
    src/localization_stage.cpp
    src/spherical_grid.cpp
    src/frame_accumulator.cpp
    src/dynamic_cloud_stage.cpp
    src/cluster_stage.cpp
)
target_link_libraries(${PROJECT_NAME}_core
    small_gicp::small_gicp
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)

# ── Offline tools library (package tools/; not runtime node path) ──
add_library(${PROJECT_NAME}_offline_lib STATIC
    tools/offline_registration.cpp
    tools/offline_pipeline.cpp
    tools/offline_detection_node.cpp
    tools/offline_test_node.cpp
)
target_link_libraries(${PROJECT_NAME}_offline_lib ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_offline_lib PUBLIC
    include
    tools
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_offline_lib
    rclcpp
    sensor_msgs
    diagnostic_msgs
    geometry_msgs
    tf2_ros
    pcl_conversions
)

# ── Node library (for testing) ─────────────────────────────────────
add_library(${PROJECT_NAME}_node_lib STATIC
    src/radar_lidar_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node_lib ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_node_lib PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_node_lib
    rclcpp
    sensor_msgs
    diagnostic_msgs
    geometry_msgs
    std_msgs
    visualization_msgs
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
)

# ── Runtime executable ─────────────────────────────────────────────
ament_auto_add_executable(${PROJECT_NAME}_node
    src/runtime.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_node_lib)

# ── Offline / CLI tools ────────────────────────────────────────────
add_executable(registration_tool
    tools/registration_tool.cpp
)
target_link_libraries(registration_tool ${PROJECT_NAME}_core)
target_include_directories(registration_tool PRIVATE include tools)

ament_auto_add_executable(offline_detection_node
    tools/offline_detection_runtime.cpp
)
target_link_libraries(offline_detection_node ${PROJECT_NAME}_offline_lib)

ament_auto_add_executable(offline_test_node
    tools/offline_test_runtime.cpp
)
target_link_libraries(offline_test_node ${PROJECT_NAME}_offline_lib)

# ── Tests ──────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(radar_tests
      test/test_map_data.cpp
      test/test_frame_accumulator.cpp
      test/test_spherical_grid.cpp
      test/test_offline_visualization.cpp
  )
  target_link_libraries(radar_tests ${PROJECT_NAME}_core ${PROJECT_NAME}_offline_lib)
  target_include_directories(radar_tests PRIVATE tools)

  ament_add_gtest(radar_core_tests
      test/test_detection.cpp
      test/test_localization.cpp
      test/test_localization_roi.cpp
  )
  target_link_libraries(radar_core_tests ${PROJECT_NAME}_core)

  ament_add_gtest(radar_surface_tests
      test/test_radar_lidar_node.cpp
  )
  target_link_libraries(radar_surface_tests ${PROJECT_NAME}_node_lib)

  ament_add_gtest(radar_registration_tests
      test/test_offline_registration.cpp
  )
  target_link_libraries(radar_registration_tests ${PROJECT_NAME}_core ${PROJECT_NAME}_offline_lib)
  target_include_directories(radar_registration_tests PRIVATE tools)
endif()

install(TARGETS ${PROJECT_NAME}_node registration_tool offline_detection_node offline_test_node
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
