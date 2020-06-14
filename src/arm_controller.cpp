// A simple program that computes the square root of a number
#include "arm_controller.h"

ArmController::ArmController(Monitor* monitorObject) {
    Eigen::Matrix4d endpose;
    monitor = monitorObject;
    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Matrix4d currEndPose = monitor->arm->getPose();
    numJoints = monitor->arm->nJoints;

    goal = (currEndPose * origin).head(3);
    objectDistance = monitor->monitorCollisionWithObjects();
    armDistances = monitor->monitorCollisionWithArm();

    for(int i=0; i<numJoints; i++) {
        jointAngles.push_back(0.0);
    }
}

ArmController::~ArmController() {
}

void ArmController::armCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // double* msgAngles = msg->position;
    // int n = sizeof(msgAngles) / sizeof(msgAngles[0]);

    // std::copy(msgAngles, msgAngles + n, jointAngles.begin());
    jointAngles.assign(msg->position.begin(), msg->position.end());

}

void ArmController::goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    //transform the message from its current type to Eigen::Vector3d and put in goal variable
}

void ArmController::controlLoop(void) {

    objectDistance = monitor->monitorCollisionWithObjects();
    armDistances = monitor->monitorCollisionWithArm();

}
