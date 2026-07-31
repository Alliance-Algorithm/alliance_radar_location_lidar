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

add_executable(annotate_l1l2l3
    annotate_l1l2l3.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
)

add_executable(annotate_l1
    annotate_l1.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
)

add_executable(annotate_l2
    annotate_l2.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
)

add_executable(annotate_l3
    annotate_l3.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
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

target_include_directories(annotate_l1l2l3 PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(annotate_l1l2l3 PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(annotate_l1l2l3
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)

target_include_directories(annotate_l1 PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(annotate_l1 PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(annotate_l1
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)

target_include_directories(annotate_l2 PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(annotate_l2 PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(annotate_l2
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)

target_include_directories(annotate_l3 PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(annotate_l3 PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(annotate_l3
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)
