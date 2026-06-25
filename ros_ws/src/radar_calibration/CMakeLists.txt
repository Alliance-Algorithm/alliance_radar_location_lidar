cmake_minimum_required(VERSION 3.24)
project(radar_calibration)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
    src/running.cpp
    src/pointcloud_process.cpp
    src/pointcloud_capture.cpp
    src/model_preprocess.cpp
    src/image_preprocess.cpp
    src/camera_lidar_calibration.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${yaml-cpp_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${PCL_INCLUDE_DIRS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_package()
