cmake_minimum_required(VERSION 3.5.0)

# Project name and version
project(Collision_Monitoring VERSION 1.0)

# Set the c++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

# Import Catch2 test framework
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

include_directories(
    src
    include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
)

add_subdirectory(CollisionMonitoring)

add_e


# List of directories to search for include files
target_include_directories(Collision_Monitoring PUBLIC
                           "${PROJECT_BINARY_DIR}/src"
                           )