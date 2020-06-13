// A simple program that computes the square root of a number
#include "arm_controller.h"

ArmController::ArmController(Monitor* monitorObject) {
    Eigen::Matrix4d endpose;
    monitor = monitorObject;
    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Matrix4d currEndPose = monitor->arm->getPose();

    goal = (currEndPose * origin).head(3);
    objectDistance = monitor->monitorCollisionWithObjects();
    armDistances = monitor->monitorCollisionWithArm();
}

ArmController::~ArmController() {
}

void ArmController::armCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    objectDistance = monitor->monitorCollisionWithObjects();
    armDistances = monitor->monitorCollisionWithArm();
    // calculations to find and then post the new velocity/arm config

}

void ArmController::goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    //transform the message from its current type to Eigen::Vector3d and put in goal variable
}
