cmake_minimum_required(VERSION 3.5.0)

# Project name and version
project(Collision_Monitoring VERSION 1.0)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

include_directories(
    src
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

# Configure config header file 
configure_file(include/collision_monitoring_config.h.in ../include/collision_monitoring_config.h)

# Executable
add_executable(Collision_Monitoring
    src/collision_monitoring.cpp
    src/primitives.cpp
    src/arm.cpp
)

add_executable(tests
    test/test.cpp
)

# src/Collision_Monitoring.cpp
# src/basic_shapes.cpp
# src/amr_model.cpp

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