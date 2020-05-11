cmake_minimum_required(VERSION 3.10)

# Project name and version
project(Collision_Monitoring VERSION 1.0)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

include_directories(
  include
  /usr/local/include
  /usr/include/eigen3
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
)

# Set the c++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

## Declare a C++ library
add_library(youbot_kdl
   src/basic_shapes.cpp
   src/arm_model.cpp
)

# Configure config header file 
configure_file(Collision_Monitoring_Config.h.in Collision_Monitoring_Config.h)

# Executable
add_executable(Collision_Monitoring
    src/Collision_Monitoring.cpp
    src/basic_shapes.cpp
    src/amr_model.cpp
)

# List of directories to search for include files
target_include_directories(Collision_Monitoring PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           )