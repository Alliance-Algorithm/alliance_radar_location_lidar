cmake_minimum_required(VERSION 3.14)
project(jinan_to_scan)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS common io)

add_executable(jinan_to_xyz jinan_to_xyz.cpp)
target_link_libraries(jinan_to_xyz ${PCL_LIBRARIES})
target_include_directories(jinan_to_xyz PRIVATE ${PCL_INCLUDE_DIRS})

add_executable(extract_by_index extract_by_index.cpp)
target_link_libraries(extract_by_index ${PCL_LIBRARIES})
target_include_directories(extract_by_index PRIVATE ${PCL_INCLUDE_DIRS})

install(TARGETS jinan_to_xyz extract_by_index DESTINATION bin)
