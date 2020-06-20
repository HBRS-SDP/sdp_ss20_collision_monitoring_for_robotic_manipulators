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
    ros::init(argc, argv, "kinova_controller");
    std::string nodeName = ros::this_node::getName();
    std::cout << "nodeName: " << nodeName << std::endl;
    ros::NodeHandle n;
    std::string modelPath;
    std::string jointStatesTopic;
    std::string goalTopic;
    std::string jointVelocityTopic;
    // setup the first monitor function
    n.param<std::string>(ros::this_node::getName()+"/urdf_model", modelPath, "./urdf/GEN3_URDF_V12.urdf");
    n.param<std::string>(ros::this_node::getName()+"/joint_state_topic", jointStatesTopic, "joint_states");
    n.param<std::string>(ros::this_node::getName()+"/goal_topic", goalTopic, ros::this_node::getName()+"/goal");
    n.param<std::string>(ros::this_node::getName()+"/velocity_topic", jointVelocityTopic, "joint_command");


    std::string model = modelPath;
    KinovaArm arm1(model);
    Monitor monitor1(&arm1);

    // Create the armController class based off the first monitor
    ArmController armController1(&monitor1, 1, 0, 100, 20/3.1425);

    // Init ROS listener
    ros::Subscriber armSub = n.subscribe(jointStatesTopic, 1000, &ArmController::armCallback, &armController1);
    ros::Subscriber goalSub = n.subscribe(goalTopic, 1000, &ArmController::goalCallback, &armController1);
    ros::Subscriber obstacleSub = n.subscribe("kinova_controller/obstacles", 1000, &ArmController::updateObstacles, &armController1);


    ros::Publisher armPub = n.advertise<sensor_msgs::JointState>(jointVelocityTopic, 1000);
    ros::Publisher markersPub = n.advertise<visualization_msgs::Marker>("kinova_controller/markers", 1000);

    
    ros::Rate loop_rate(100); 


    KDL::Twist endeffectorVelocity;
    std::vector<double> jointVelocities;
    sensor_msgs::JointState jointStates;
    for (int i=0; i<arm1.nJoints; i++) {
        jointStates.position.push_back(0.0);
        jointStates.velocity.push_back(0.0);
    }


    while(ros::ok()) {
        endeffectorVelocity = armController1.controlLoop();
        std::cout << "end vel: "<<endeffectorVelocity <<std::endl;
        jointVelocities = arm1.ikVelocitySolver(endeffectorVelocity);
        std::cout << "joint Vels: ";

        for(int i=0; i<arm1.nJoints; i++){
            std::cout << jointVelocities[i] << " ";
            jointStates.velocity[i] = jointVelocities[i];
            jointStates.position[i] = arm1.jointArray(i);
        }
        std::cout<<std::endl;
        std::cout<< arm1.getPose() << std::endl;
        armPub.publish(jointStates);
        for(int i=0; i<armController1.rvizObstacles.size(); i++){
            markersPub.publish(armController1.rvizObstacles[i]->marker);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}