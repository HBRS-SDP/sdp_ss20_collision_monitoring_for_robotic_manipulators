cmake_minimum_required(VERSION 3.5.0)

# Project name and version
project(Kinova_Collision_Monitoring VERSION 1.0)

# Set the c++ version
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(collision_monitoring)
add_subdirectory(src)
add_subdirectory(test)
