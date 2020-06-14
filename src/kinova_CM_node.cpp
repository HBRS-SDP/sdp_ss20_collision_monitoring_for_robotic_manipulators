// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
#include "primitives.h"
#include "ros/ros.h"
#include "arm_controller.h"
#include "sensor_msgs/JointState.h"


int main(int argc, char **argv)
{
    // setup the first monitor function
    std::string urdf_filename1 = "../urdf/GEN3_URDF_V12.urdf";
    KinovaArm arm1(urdf_filename1);
    Monitor monitor1(&arm1);

    // Create the armController class based off the first monitor
    ArmController armController1(&monitor1);

    // Init ROS listener
    ros::init(argc, argv, "kinova_controller");
    ros::NodeHandle n;
    ros::Subscriber armSub = n.subscribe("my_gen3/joint_states", 1000, &ArmController::armCallback, &armController1);
    ros::Subscriber goalSub = n.subscribe("kinova_controller/goal", 1000, &ArmController::goalCallback, &armController1);

    ros::Publisher armPub = n.advertise<sensor_msgs::JointState>("kinova_controller/joint_commands", 1000);
    
    ros::Rate loop_rate(10);


    std::vector<double> jointVelocities;

    while(ros::ok()) {
        jointVelocities = armController1.controlLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}