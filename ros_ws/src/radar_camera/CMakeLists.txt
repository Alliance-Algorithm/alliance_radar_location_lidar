cmake_minimum_required(VERSION 3.24)
project(radar_camera)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")
if(NOT CMAKE_CROSSCOMPILING)
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c "from openvino.utils import get_cmake_path; print(get_cmake_path(), end='')"
            OUTPUT_VARIABLE OpenVINO_DIR_PY
            ERROR_QUIET)
    endif()
endif()

find_package(OpenVINO REQUIRED PATHS "${OpenVINO_DIR_PY}")
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn calib3d)
find_package(Eigen3 REQUIRED)
find_package(assimp REQUIRED)

ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME}_core STATIC
    src/model_inference.cpp
    src/projector.cpp
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
    PUBLIC
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        opencv_dnn
        opencv_calib3d
        openvino::runtime
        Eigen3::Eigen
        assimp::assimp
)

ament_auto_add_executable(${PROJECT_NAME}_node
    src/runtime.cpp
    src/radar_camera_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(radar_camera_tests
      test/test_model_inference.cpp
      test/test_projector.cpp
  )
  target_link_libraries(radar_camera_tests ${PROJECT_NAME}_core)
  target_compile_definitions(radar_camera_tests PRIVATE
      RADAR_CAMERA_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test"
  )
endif()

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
ament_package()
