cmake_minimum_required(VERSION 3.10)

# Project name and version
project(Collision_Monitoring VERSION 1.0)

# Set the c++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Configure config header file 
configure_file(Collision_Monitoring_Config.h.in Collision_Monitoring_Config.h)

# Executable
add_executable(Collision_Monitoring Collision_Monitoring.cpp)

# List of directories to search for include files
target_include_directories(Collision_Monitoring PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           )