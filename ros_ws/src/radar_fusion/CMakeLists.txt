cmake_minimum_required(VERSION 3.24)
project(radar_fusion)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
    src/runtime.cpp
    src/fusion_node.cpp
    src/kalman_tracker.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_package()
