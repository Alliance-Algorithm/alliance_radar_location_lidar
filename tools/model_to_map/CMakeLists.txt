cmake_minimum_required(VERSION 3.14)
project(model_to_map)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(assimp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)

add_executable(model_to_map src/model_to_map.cpp)

target_link_libraries(model_to_map
    assimp::assimp
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    rt
)

target_include_directories(model_to_map PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
)

install(TARGETS model_to_map DESTINATION bin)
