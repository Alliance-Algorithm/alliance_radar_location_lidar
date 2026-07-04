cmake_minimum_required(VERSION 3.24)
project(radar_calibration)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)

ament_auto_find_build_dependencies()

# ── Core library ─────────────────────────────────────────────────
add_library(${PROJECT_NAME}_core STATIC
    src/camera_lidar_calibration.cpp
)
target_link_libraries(${PROJECT_NAME}_core
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    ${yaml-cpp_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    include
    ${yaml-cpp_INCLUDE_DIRS}
)

# ── CLI entry point ────────────────────────────────────────────────
add_executable(${PROJECT_NAME}_node
    src/running.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

# ── Tests ────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(radar_calibration_tests
      test/test_camera_lidar_calibration.cpp
  )
  target_link_libraries(radar_calibration_tests ${PROJECT_NAME}_core)
endif()

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_package()
