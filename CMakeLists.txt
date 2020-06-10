cmake_minimum_required(VERSION 2.8.3)

# Project name and version
project(Kinova_Collision_Monitoring)

# Set the c++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Debug)

add_subdirectory(collision_monitoring)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)

include_directories(
    include
    collision_monitoring/include
    /usr/local/include
    /usr/local/lib
    /usr/include/eigen3
    /opt/ros/kinetic/include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
)

# set the path to the library folder
link_directories(/usr/local/lib)

add_library(KinovaArm STATIC
    src/kinova_arm.cpp
    include/kinova_arm.h
)

target_include_directories(KinovaArm PUBLIC
                           "${PROJECT_BINARY_DIR}"
)

add_executable(KinovaCollisionMonitoring
    src/arm_controller.cpp
    src/kinova_arm.cpp
)

target_link_libraries(KinovaCollisionMonitoring PRIVATE
    KinovaArm
    CollisionMonitoring
    orocos-kdl
    kdl_parser
    ${orocos-kdl_LIBRARIES}
)

add_subdirectory(test)