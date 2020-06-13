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

void armCallback_0(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i=0; i<msg->position.size() ; ++i)
    {
        ROS_INFO_STREAM("robot 0: " << msg->name[i] << " " << msg->position[i]);
    }
}

int main(int argc, char **argv)
{
    std::string urdf_filename1 = "../urdf/GEN3_URDF_V12.urdf";
    KinovaArm arm1(urdf_filename1);
    Monitor monitor1(&arm1);

    ArmController armController1(&monitor1);

    ros::init(argc, argv, "pose_listener");
    ros::NodeHandle n;
    ros::Subscriber sub_0 = n.subscribe("my_gen3_0/joint_states", 1000, &ArmController::armCallback, &armController1);
    ros::spin();

    return 0;
}