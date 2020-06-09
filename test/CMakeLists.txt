project(cmake_test)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)

set(CMAKE_BUILD_TYPE Debug)

include_directories(
    ../include
    ../collision_monitoring/include
    /opt/ros/kinetic/include
    /usr/local/include
    ${Eigen_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)

# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})


set(KINOVA_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../src/kinova_arm.cpp)
# Make test executable
set(TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp)
add_executable(tests ${TEST_SOURCES} ${KINOVA_SOURCES})
target_link_libraries(tests Catch KinovaArm CollisionMonitoring kdl_parser)
