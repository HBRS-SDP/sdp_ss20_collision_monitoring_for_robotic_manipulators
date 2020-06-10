cmake_minimum_required(VERSION 3.5.0)

project(Collision_Monitoring)

# Set the c++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
)

add_library(CollisionMonitoring
    src/primitives.cpp
    src/monitor.cpp
    src/arm.cpp
)

target_link_libraries(CollisionMonitoring
    orocos-kdl
    ${orocos-kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
)

target_include_directories(CollisionMonitoring INTERFACE
                           "${CMAKE_CURRENT_SOURCE_DIR}/include"
)