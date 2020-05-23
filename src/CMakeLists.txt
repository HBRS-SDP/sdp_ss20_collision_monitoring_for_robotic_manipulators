find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

include_directories(
    ../include
    ../collision_monitoring/include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
)

add_library(KinovaArm STATIC
    kinova_arm.cpp
    kinova_arm.h
)

target_include_directories(KinovaArm PUBLIC
                           "${PROJECT_BINARY_DIR}"
)

add_executable(KinovaCollisionMonitoring
    collision_monitoring.cpp
)

target_link_libraries(KinovaCollisionMonitoring PRIVATE
    KinovaArm
    CollisionMonitoring
    orocos-kdl
    ${orocos-kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
)