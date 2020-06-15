// A simple program that computes the square root of a number
#include "arm_controller.h"

ArmController::ArmController(Monitor* monitorObject) {
    Eigen::Matrix4d endpose;
    monitor = monitorObject;
    Eigen::Matrix4d currEndPose = monitor->arm->getPose();
    numJoints = monitor->arm->nJoints;

    origin << 0, 0, 0, 1;

    goal = (currEndPose * origin).head(3);
    objectDistances = monitor->distanceToObjects();
    armDistances = monitor->distanceBetweenArmLinks();

    for(int i=0; i<numJoints; i++) {
        jointAngles.push_back(0.0);
    }
}

ArmController::~ArmController() {
    for (int i=0; i<rvizObstacles.size();i++) {
        delete(rvizObstacles[i]);
    }
}

void ArmController::armCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // copy the joint angles
    std::cout << "armCallback:\n";
    for (int i=0; i<msg->position.size(); i++) {
        std::cout << msg->position[i] << std::endl;
    }
    jointAngles.assign(msg->position.begin(), msg->position.end());

}

void ArmController::goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    //transform the message from its current type to Eigen::Vector3d and put in goal variable
    std::cout << "goalCallback:\n\tCurrent goal: "<<this->goal<<std::endl;
    std::cout << "\tincoming goal: " << msg->x << ", " << msg->y << ", " << msg->z << std::endl;
    this->goal[0] = msg->x;
    this->goal[1] = msg->y;
    this->goal[2] = msg->z;
    std::cout << "\tNew goal: "<<this->goal<<std::endl;
}

std::vector<double> ArmController::controlLoop(void) {
    // Variable for storing the resulting joint velocities
    std::vector<double> jointVelocities;
    std::cout << 1 <<std::endl;
    // Update the current state to match real arm state
    this->monitor->arm->updatePose(this->jointAngles);
    std::cout << 2 <<std::endl;
    Eigen::Matrix4d currEndPose = monitor->arm->getPose();
    std::cout << 3 <<std::endl;
    Eigen::Vector3d currEndPoint = (currEndPose * origin).head(3);
    std::cout << 4 <<std::endl;
    objectDistances = monitor->distanceToObjects();
    std::cout << 5 <<std::endl;
    armDistances = monitor->distanceBetweenArmLinks();
    std::cout << 6 <<std::endl;

    // TODO the code for the object avoidance

    return jointVelocities;
}

void ArmController::updateObstacles(const visualization_msgs::Marker::ConstPtr& msg) {
    std::cout << "New obstacle of type: " << msg->type <<std::endl;
    if (msg->type == visualization_msgs::Marker::SPHERE) {
        bool newObstacle = true;
        for(int i=0; i<rvizObstacles.size();i++){
            if(rvizObstacles[i]->marker.id == msg->id) {
                newObstacle = false;
                int index = rvizObstacles[i]->idx;
                monitor->obstacles[index]->pose = rvizObstacles[i]->updatePose(msg);
            }
        }

        if(newObstacle) {
            RvizObstacle* rvizObstacle = new RvizObstacle(msg, rvizObstacles.size());
            rvizObstacles.push_back(rvizObstacle);
            Sphere sphere(rvizObstacle->pose, rvizObstacle->marker.scale.x);
            monitor->addObstacle(&sphere);
        }
    }
    else {
        ROS_ERROR("Wrong shape for obstacle");
    }
}


RvizObstacle::RvizObstacle(visualization_msgs::Marker::ConstPtr markerIn, int index) {
    marker = *markerIn;
    idx = index;
    double qx = marker.pose.orientation.x;
    double qy = marker.pose.orientation.y;
    double qz = marker.pose.orientation.z;
    double qw = marker.pose.orientation.w;
    double px = marker.pose.position.x;
    double py = marker.pose.position.y;
    double pz = marker.pose.position.z;
    pose << 1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw, px,
            2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw, py,
            2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy, pz,
            0, 0, 0, 1;
}

RvizObstacle::~RvizObstacle() {
}

Eigen::Matrix4d RvizObstacle::updatePose(visualization_msgs::Marker::ConstPtr markerIn) {
    marker = *markerIn;
    double qx = marker.pose.orientation.x;
    double qy = marker.pose.orientation.y;
    double qz = marker.pose.orientation.z;
    double qw = marker.pose.orientation.w;
    double px = marker.pose.position.x;
    double py = marker.pose.position.y;
    double pz = marker.pose.position.z;
    pose << 1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw, px,
            2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw, py,
            2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy, pz,
            0, 0, 0, 1;
    return pose;
}




