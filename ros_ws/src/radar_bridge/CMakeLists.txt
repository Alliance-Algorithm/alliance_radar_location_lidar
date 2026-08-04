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
find_package(hikcamera REQUIRED)
find_package(radar_interfaces REQUIRED)

set(RADAR_SRC /workspace/ros_ws/src/radar_camera)
set(TRT_ROOT /opt/radar_camera_trt)
set(CUDA_ROOT ${TRT_ROOT})

ament_auto_find_build_dependencies()
set(SOURCE_FILES
    src/radar_bridge_node.cpp
    src/zmq_bridge.cpp
    src/videostream_bridge.cpp
    src/runtime.cpp
)
ament_auto_add_executable(${PROJECT_NAME}_node
    ${SOURCE_FILES}
)
add_executable(test_zmq_send
    tests/test_zmq_send.cpp
    src/zmq_bridge.cpp
)
target_include_directories(test_zmq_send PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_zmq_send
    cppzmq
    nlohmann_json::nlohmann_json
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
)

target_sources(${PROJECT_NAME}_node PRIVATE
    ${RADAR_SRC}/src/armor_infer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    # CUDA kernel 预编译对象（与 radar_camera 一致，sm_89 = 比赛机 RTX 4060）
    ${RADAR_SRC}/cuda_prebuilt/image_normalize.sm89.o
)

target_link_directories(${PROJECT_NAME}_node PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(${PROJECT_NAME}_node
    hikcamera::hikcamera
    cppzmq
    ${OpenCV_LIBRARIES}
    nlohmann_json::nlohmann_json
    nvinfer
    nvinfer_plugin
    cudart
)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_package()
