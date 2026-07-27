cmake_minimum_required(VERSION 3.24)
project(radar_fast_livo2_rgb)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(PCL REQUIRED)

# ── ROS2 dependencies for camera recorder ──
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Live camera recorder node (requires hikcamera SDK) ──
#
# The recorder executable consumes HIK frames via the installed hikcamera
# SHM API.  When hikcamera is not installed (e.g. isolated worktree without
# OpenCV 4.5), set BUILD_CAMERA_RECORDER=OFF explicitly.  The default is ON
# so a missing dependency produces a clear CMake error rather than silently
# skipping the source.
#
#   cmake -DBUILD_CAMERA_RECORDER=OFF ..
#
option(BUILD_CAMERA_RECORDER "Build the live camera recorder node (needs hikcamera)" ON)

# ════════════════════════════════════════════════════════════════════════
# Camera recorder library (pure helpers, no ROS node dependency)
# ════════════════════════════════════════════════════════════════════════

add_library(camera_frame_lib INTERFACE)
target_include_directories(camera_frame_lib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(camera_frame_lib INTERFACE
  opencv_core
)
ament_target_dependencies(camera_frame_lib INTERFACE
  builtin_interfaces
  sensor_msgs
)

# ════════════════════════════════════════════════════════════════════════
# Camera recorder node executable
# ════════════════════════════════════════════════════════════════════════

if(BUILD_CAMERA_RECORDER)
  find_package(hikcamera REQUIRED)

  # ── RGB core/static library ──
  add_library(color_voxel_map STATIC
    src/color_voxel_map.cpp
    src/rgb_pcd_io.cpp)
  target_link_libraries(color_voxel_map PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
  target_include_directories(color_voxel_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS})

  # ── Camera recorder node ──
  add_executable(camera_recorder_node src/camera_recorder_node.cpp)
  target_link_libraries(camera_recorder_node
    camera_frame_lib
    opencv_core
    opencv_imgproc
    hikcamera::hikcamera
  )
  target_include_directories(camera_recorder_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(camera_recorder_node
    rclcpp
    sensor_msgs
    builtin_interfaces
  )

  install(TARGETS camera_recorder_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # ── RGB colorizer node (Task 5) ──
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)

  add_executable(rgb_colorizer_node src/rgb_colorizer_node.cpp)
  target_link_libraries(rgb_colorizer_node
    camera_frame_lib
    color_voxel_map
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    hikcamera::hikcamera
    ${PCL_LIBRARIES}
  )
  target_include_directories(rgb_colorizer_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(rgb_colorizer_node
    rclcpp
    sensor_msgs
    builtin_interfaces
    nav_msgs
    geometry_msgs
    pcl_conversions
  )

  install(TARGETS rgb_colorizer_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
else()
  message(STATUS "Camera recorder & colorizer nodes disabled (BUILD_CAMERA_RECORDER=OFF)")
endif()

# ════════════════════════════════════════════════════════════════════════
# Tests
# ════════════════════════════════════════════════════════════════════════

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rgb_projection test/test_rgb_projection.cpp)
  target_link_libraries(test_rgb_projection
    Eigen3::Eigen
    ${OpenCV_LIBS}
  )
  target_include_directories(test_rgb_projection
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(test_rgb_pcd_io
    test/test_rgb_pcd_io.cpp
    src/rgb_pcd_io.cpp)
  target_include_directories(test_rgb_pcd_io PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # When building on a system without real PCL, set PCL_STUB_DIR to the
  # stub headers path (see task-2-report.md for details).
  if(PCL_STUB_DIR)
    add_library(pcl_stub STATIC ${CMAKE_CURRENT_SOURCE_DIR}/test/stubs/pcd_io_stub.cpp)
    target_include_directories(pcl_stub PUBLIC ${PCL_STUB_DIR}/include)
  endif()

  # NOTE: test_color_voxel_map requires full PCL linkage (including
  # pcl_io).  When building outside the Docker container (e.g. isolated
  # worktree), set SKIP_COLOR_VOXEL_TEST=ON to skip it.
  option(SKIP_COLOR_VOXEL_TEST "Skip color voxel map tests (needs full PCL)" OFF)
  if(NOT SKIP_COLOR_VOXEL_TEST)
    ament_add_gtest(test_color_voxel_map test/test_color_voxel_map.cpp
      src/color_voxel_map.cpp)
    target_link_libraries(test_color_voxel_map
      Eigen3::Eigen
      ${PCL_LIBRARIES}
    )
    if(PCL_STUB_DIR)
      target_link_libraries(test_color_voxel_map pcl_stub)
    endif()
    target_include_directories(test_color_voxel_map
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PCL_INCLUDE_DIRS}
    )
  endif()

  # ── Task 4: camera-frame helpers test ──
  ament_add_gtest(test_camera_frame test/test_camera_frame.cpp)
  target_link_libraries(test_camera_frame
    camera_frame_lib
    opencv_core
    opencv_imgproc
  )
  target_include_directories(test_camera_frame PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # ── Task 5: RGB colorizer integration test ──
  ament_add_gtest(test_rgb_colorizer test/test_rgb_colorizer.cpp
    src/color_voxel_map.cpp)
  target_link_libraries(test_rgb_colorizer
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
  )
  if(PCL_STUB_DIR)
    target_link_libraries(test_rgb_colorizer pcl_stub)
  endif()
  target_include_directories(test_rgb_colorizer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  )
endif()

ament_export_dependencies(
  builtin_interfaces
  sensor_msgs
)

ament_package()
