project(cmake_test)

include_directories(
    ../include
    ../collision_monitoring/include
    /usr/local/include
    ${Eigen_INCLUDE_DIRS}
)

# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

# Make test executable
set(TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test.cpp)
add_executable(tests ${TEST_SOURCES})
target_link_libraries(tests Catch)

set(TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test_cylinder.cpp)
add_executable(test_cylinder ${TEST_SOURCES})
target_link_libraries(test_cylinder Catch CollisionMonitoring)
