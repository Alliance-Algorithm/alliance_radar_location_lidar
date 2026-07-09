cmake_minimum_required(VERSION 3.24)
project(radar_bridge)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cppzmq REQUIRED)
find_package(radar_interfaces REQUIRED)

ament_auto_find_build_dependencies()
set(SOURCE_FILES
    src/radar_bridge_node.cpp
    src/zmq_bridge.cpp
    src/udpstream_bridge.cpp
    src/runtime.cpp
)
ament_auto_add_executable(${PROJECT_NAME}_node
    ${SOURCE_FILES}
)
target_link_libraries(${PROJECT_NAME}_node
    cppzmq
    ${OpenCV_LIBRARIES}
    nlohmann_json::nlohmann_json
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_package()
