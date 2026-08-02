cmake_minimum_required(VERSION 3.16)
project(mp4_replay CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

set(RADAR_WS /workspace)
set(HIK_INSTALL ${RADAR_WS}/ros_ws/install/hikcamera)

add_executable(mp4_replay mp4_replay.cpp)

target_include_directories(mp4_replay PRIVATE
    ${HIK_INSTALL}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(mp4_replay PRIVATE
    ${HIK_INSTALL}/lib
)

target_link_libraries(mp4_replay
    ${OpenCV_LIBS}
    hikcamera
    pthread
    rt
)

install(TARGETS mp4_replay DESTINATION bin)
