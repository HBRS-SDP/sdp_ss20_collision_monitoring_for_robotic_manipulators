// A simple program that computes the square root of a number
#include "arm_controller.h"

CollisionMonitor::CollisionMonitor(Monitor monitorObject){
    Eigen::Matrix4d endpose;
    Eigen::Matrix4d origin(0, 0, 0, 1);
    goal = this->monitor.arm.getPose() * origin;
    objectDistance = monitor.monitorCollisionWithObjects();
    amrDistances = monitor.monitorCollisionWithArm();
}

void CollisionMonitor::armCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    objectDistance = monitor.monitorCollisionWithObjects();
    amrDistances = monitor.monitorCollisionWithArm();
    // calculations to find and then post the new velocity/arm config

}

void CollisionMonitor::goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    //transform the message from its current type to Eigen::Vector3d and put in goal variable
}