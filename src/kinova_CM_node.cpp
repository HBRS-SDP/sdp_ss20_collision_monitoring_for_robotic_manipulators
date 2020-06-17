// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
#include "primitives.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "arm_controller.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"


int main(int argc, char **argv)
{
    // setup the first monitor function
    std::string urdf_filename1 = "./urdf/GEN3_URDF_V12.urdf";
    KinovaArm arm1(urdf_filename1);
    Monitor monitor1(&arm1);

    // Create the armController class based off the first monitor
    ArmController armController1(&monitor1, 1, 0, 100, 20/3.1425);

    // Init ROS listener
    ros::init(argc, argv, "kinova_controller");
    ros::NodeHandle n;
    ros::Subscriber armSub = n.subscribe("my_gen3/joint_states", 1000, &ArmController::armCallback, &armController1);
    ros::Subscriber goalSub = n.subscribe("kinova_controller/goal", 1000, &ArmController::goalCallback, &armController1);
    ros::Subscriber obstacleSub = n.subscribe("kinova_controller/obstacles", 1000, &ArmController::updateObstacles, &armController1);


    ros::Publisher armPub = n.advertise<sensor_msgs::JointState>("kinova_controller/joint_commands", 1000);
    ros::Publisher markersPub = n.advertise<visualization_msgs::Marker>("kinova_controller/markers", 1000);
    
    ros::Rate loop_rate(1.5);


    KDL::Twist endeffectorVelocity;
    std::vector<double> jointVelocities;
    sensor_msgs::JointState jointStates;
    for (int i=0; i<arm1.nJoints; i++) {
        jointStates.velocity.push_back(0.0);
    }


    while(ros::ok()) {
        endeffectorVelocity = armController1.controlLoop();
        jointVelocities = arm1.ikVelocitySolver(endeffectorVelocity);
        for(int i=0; i<arm1.nJoints; i++){
            jointStates.velocity[i] = jointVelocities[i];
        }
        armPub.publish(jointStates);
        for(int i=0; i<armController1.rvizObstacles.size(); i++){
            markersPub.publish(armController1.rvizObstacles[i]->marker);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}