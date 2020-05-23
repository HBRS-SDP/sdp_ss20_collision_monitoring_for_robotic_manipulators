add_library(CollisionMonitoring
arm.cpp
primatives.cpp
)

target_link_libraries(Collision_Monitoring
    orocos-kdl
    ${orocos-kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
)