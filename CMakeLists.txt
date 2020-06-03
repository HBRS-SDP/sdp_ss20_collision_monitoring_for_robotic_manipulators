cmake_minimum_required(VERSION 3.5.0)

# Project name and version
project(Kinova_Collision_Monitoring VERSION 1.0)

# Set the c++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(collision_monitoring)
add_subdirectory(test)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

include_directories(
    include
    /collision_monitoring/include
    /usr/local/include
    /usr/include/eigen3
    /opt/ros/kinetic/include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
)

add_library(KinovaArm STATIC
    src/kinova_arm.cpp
    include/kinova_arm.h
)

target_include_directories(KinovaArm PUBLIC
                           "${PROJECT_BINARY_DIR}"
)

add_executable(KinovaCollisionMonitoring
    src/collision_monitoring.cpp
    src/kinova_arm.cpp
)

target_link_libraries(KinovaCollisionMonitoring PRIVATE
    KinovaArm
    CollisionMonitoring
    orocos-kdl
    ${orocos-kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
)