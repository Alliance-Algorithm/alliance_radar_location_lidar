cmake_minimum_required(VERSION 3.16)
project(armor_verify CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

set(RADAR_SRC /workspace/ros_ws/src/radar_camera)
set(TRT_ROOT  /opt/radar_camera_trt)
set(CUDA_ROOT /usr/local/cuda)

add_executable(validate_3layer
    validate_3layer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${RADAR_SRC}/src/armor_refine.cpp
)

target_include_directories(validate_3layer PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(validate_3layer PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(validate_3layer
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)
