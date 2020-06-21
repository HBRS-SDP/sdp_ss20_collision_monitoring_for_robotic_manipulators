// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"

#include "primitives.h"
#include "arm_controller.h"

geometry_msgs::TransformStamped generateTF(Eigen::Matrix4d baseTransform, std::string base_name) {
    geometry_msgs::TransformStamped static_transformStamped;
    double w = sqrt(1 + baseTransform(0,0)
                       + baseTransform(1,1) 
                       + baseTransform(2, 2))/2;
    double x = (baseTransform(2,1) - baseTransform(1, 2))/(4*w);
    double y = (baseTransform(0,2) - baseTransform(2, 0))/(4*w);
    double z = (baseTransform(1,0) - baseTransform(0, 1))/(4*w);
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "world";
    static_transformStamped.child_frame_id = base_name;
    static_transformStamped.transform.translation.x = baseTransform(0,3);
    static_transformStamped.transform.translation.y = baseTransform(1,3);
    static_transformStamped.transform.translation.z = baseTransform(2,3);
    tf2::Quaternion quat(x, y, z, w);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    return static_transformStamped;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinova_controller");
    std::string nodeName = ros::this_node::getName();
    std::cout << "nodeName: " << nodeName << std::endl;
    ros::NodeHandle n;
    std::string modelPath1;
    std::string jointStatesTopic1;
    std::string goalTopic1;
    std::string jointVelocityTopic1;
    std::string armNameSpace1;
    std::string modelPath2;
    std::string jointStatesTopic2;
    std::string goalTopic2;
    std::string jointVelocityTopic2;
    std::string armNamespace2;
    // setup the first monitor function
    n.param<std::string>(ros::this_node::getName()+"/urdf_model", modelPath, "./urdf/GEN3_URDF_V12.urdf");
    n.param<std::string>(ros::this_node::getName()+"/joint_state_topic_1", jointStatesTopic1, "joint_states_1");
    n.param<std::string>(ros::this_node::getName()+"/joint_state_topic_2", jointStatesTopic2, "joint_states_2");
    n.param<std::string>(ros::this_node::getName()+"/goal_topic_1", goalTopic1, ros::this_node::getName()+"/goal_1");
    n.param<std::string>(ros::this_node::getName()+"/goal_topic_2", goalTopic2, ros::this_node::getName()+"/goal_2");
    n.param<std::string>(ros::this_node::getName()+"/velocity_topic_1", jointVelocityTopic1, "joint_command_1");
    n.param<std::string>(ros::this_node::getName()+"/velocity_topic_2", jointVelocityTopic2, "joint_command_2");
    n.param<std::string>(ros::this_node::getName()+"/set_arm_1_namespace", armNameSpace1, "/kinova_1");
    n.param<std::string>(ros::this_node::getName()+"/set_arm_2_namespace", armNameSpace2, "/kinova_2");


    std::string model = modelPath;
    Eigen::Matrix4d baseTransform1;
    Eigen::Matrix4d baseTransform2;
    baseTransform1 << 1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, 1, 0,
                      0, 0, 0, 1;

    baseTransform2 << 1, 0, 0, 0,
                      0, 1, 0, 1,
                      0, 0, 1, 0,
                      0, 0, 0, 1;

    KinovaArm arm1(model, baseTransform1);
    KinovaArm arm2(model, baseTransform2);
    Monitor monitor1(&arm1);
    monitor1.addObstacle(&arm2);
    std::vector<double> initPose = {0, 0, 0, 0, 0, 0, 0};
    arm1.updatePose(initPose);

    // Create the armController class based off the first monitor
    ArmController armController1(&monitor1, 1, 0, 100, 20/3.1425);

    // Init ROS listener
    ros::Subscriber armSub = n.subscribe(jointStatesTopic, 1000, &ArmController::armCallback, &armController1);
    ros::Subscriber goalSub = n.subscribe(goalTopic, 1000, &ArmController::goalCallback, &armController1);
    ros::Subscriber obstacleSub = n.subscribe("kinova_controller/obstacles", 1000, &ArmController::updateObstacles, &armController1);
    tf2_ros::StaticTransformBroadcaster base_broadcaster;
    base_broadcaster.sendTransform(generateTF(baseTransform1, armNameSpace1+"/base_link"));
    base_broadcaster.sendTransform(generateTF(baseTransform2, armNameSpace2+"/base_link"));


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