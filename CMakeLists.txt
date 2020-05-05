cmake_minimum_required(VERSION 3.5.1)

# Project name and version
project(Collision_Monitoring VERSION 1.0)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  /usr/local/include
  /usr/include/eigen3
  ${Eigen_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

# Set the c++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Configure config header file 
configure_file(src/Collision_Monitoring_Config.h.in ../src/Collision_Monitoring_Config.h)

# Executable
add_executable(Collision_Monitoring src/Collision_Monitoring.cpp)

# add kdl library
link_directories("/usr/local/include/kdl")

target_link_libraries(Collision_Monitoring
    orocos-kdl
    ${orocos-kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
)

# List of directories to search for include files
target_include_directories(Collision_Monitoring PUBLIC
                           "${PROJECT_BINARY_DIR}/src"
                           )