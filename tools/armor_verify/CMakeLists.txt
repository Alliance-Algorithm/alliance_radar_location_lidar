cmake_minimum_required(VERSION 3.16)
project(armor_verify CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
# host lacks VTK/HDF5; link only the modules these tools actually use
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_imgcodecs opencv_dnn opencv_videoio)

if(NOT DEFINED RADAR_SRC)
    set(RADAR_SRC /workspace/ros_ws/src/radar_camera)
endif()
if(NOT DEFINED TRT_ROOT)
    set(TRT_ROOT /opt/radar_camera_trt)
endif()
set(CUDA_ROOT ${TRT_ROOT})
set(CUDA_PREBUILT_OBJ ${RADAR_SRC}/cuda_prebuilt/image_normalize.sm89.o)

add_executable(validate_3layer
    validate_3layer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${RADAR_SRC}/src/armor_refine.cpp
    ${CUDA_PREBUILT_OBJ}
)

add_executable(annotate_l1l2l3
    annotate_l1l2l3.cpp
    ${RADAR_SRC}/src/armor_infer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${CUDA_PREBUILT_OBJ}
)

add_executable(annotate_l1
    annotate_l1.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${CUDA_PREBUILT_OBJ}
)

add_executable(annotate_l2
    annotate_l2.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${CUDA_PREBUILT_OBJ}
)

add_executable(annotate_l3
    annotate_l3.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${CUDA_PREBUILT_OBJ}
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

add_executable(test_armor_infer
    test_armor_infer.cpp
    ${RADAR_SRC}/src/armor_infer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${CUDA_PREBUILT_OBJ}
)

target_include_directories(test_armor_infer PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(test_armor_infer PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_compile_options(test_armor_infer PRIVATE -UNDEBUG)

target_link_libraries(test_armor_infer
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)

add_executable(video_annotate
    video_annotate.cpp
    ${RADAR_SRC}/src/armor_infer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${CUDA_PREBUILT_OBJ}
)

target_include_directories(video_annotate PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(video_annotate PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(video_annotate
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)

add_executable(compare_preprocess
    compare_preprocess.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
    ${RADAR_SRC}/src/armor_infer.cpp
    ${CUDA_PREBUILT_OBJ}
)

target_include_directories(compare_preprocess PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(compare_preprocess PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(compare_preprocess
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)
