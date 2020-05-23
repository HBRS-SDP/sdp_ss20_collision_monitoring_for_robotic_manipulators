include_directories(
    ../include
    ../collision_monitoring/include
)

enable_testing()
add_executable(tests test.cpp)
add_test(Tests tests)