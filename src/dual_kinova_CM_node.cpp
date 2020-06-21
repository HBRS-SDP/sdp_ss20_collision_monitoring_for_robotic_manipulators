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
    ros::NodeHandle n1;
    ros::NodeHandle n2;
    std::string modelPath;
    std::string jointStatesTopic;
    std::string goalTopic;
    std::string jointVelocityTopic;
    std::string obstacleTopic;
    std::string armNameSpace1;
    std::string armNameSpace2;
    // setup the first monitor function
    n1.param<std::string>(ros::this_node::getName()+"/urdf_model", modelPath, "./urdf/GEN3_URDF_V12.urdf");
    n1.param<std::string>(ros::this_node::getName()+"/joint_state_topic", jointStatesTopic, "/joint_states");
    n1.param<std::string>("/goal_topic", goalTopic, "/goal");
    n1.param<std::string>(ros::this_node::getName()+"/velocity_topic", jointVelocityTopic, "/joint_command");
    n1.param<std::string>("/obstacle_topic", obstacleTopic, "/obstacles_update");
    n1.param<std::string>("/set_arm_1_namespace", armNameSpace1, "/kinova_1");
    n1.param<std::string>("/set_arm_2_namespace", armNameSpace2, "/kinova_2");


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
    Monitor monitor2(&arm2);
    // monitor1.addObstacle(&arm2);
    // monitor2.addObstacle(&arm1);
    std::vector<double> initPose = {0, 0, 0, 0, 0, 0, 0};
    arm1.updatePose(initPose);

    // Create the armController class based off the first monitor
    ArmController armController1(&monitor1, 1, 0, 100, 20/3.1425);
    ArmController armController2(&monitor2, 1, 0, 100, 20/3.1425);

    // Init ROS listeners for first arm
    ros::Subscriber armSub1 = n1.subscribe(armNameSpace1+jointStatesTopic, 1000, &ArmController::armCallback, &armController1);
    ros::Subscriber goalSub1 = n1.subscribe(armNameSpace1+goalTopic, 1000, &ArmController::goalCallback, &armController1);
    ros::Subscriber obstacleSub1 = n1.subscribe(obstacleTopic, 1000, &ArmController::updateObstacles, &armController1);

    // Init ROS listeners for second arm
    ros::Subscriber armSub2 = n2.subscribe(armNameSpace2+jointStatesTopic, 1000, &ArmController::armCallback, &armController2);
    ros::Subscriber goalSub2 = n2.subscribe(armNameSpace2+goalTopic, 1000, &ArmController::goalCallback, &armController2);
    ros::Subscriber obstacleSub2 = n2.subscribe(obstacleTopic, 1000, &ArmController::updateObstacles, &armController2);

    // Publish static positions
    tf2_ros::StaticTransformBroadcaster base_broadcaster;
    base_broadcaster.sendTransform(generateTF(baseTransform1, armNameSpace1+"/base_link"));
    base_broadcaster.sendTransform(generateTF(baseTransform2, armNameSpace2+"/base_link"));


    ros::Publisher armPub1 = n1.advertise<sensor_msgs::JointState>(armNameSpace1 + jointVelocityTopic, 1000);
    ros::Publisher armPub2 = n2.advertise<sensor_msgs::JointState>(armNameSpace2 + jointVelocityTopic, 1000);
    ros::Publisher markersPub = n1.advertise<visualization_msgs::Marker>("kinova_controller/markers", 1000);

    
    ros::Rate loop_rate(100); 


    KDL::Twist endeffectorVelocity1;
    std::vector<double> jointVelocities1;
    sensor_msgs::JointState jointStates1;
    for (int i=0; i<arm1.nJoints; i++) {
        jointStates1.position.push_back(0.0);
        jointStates1.velocity.push_back(0.0);
    }

    KDL::Twist endeffectorVelocity2;
    std::vector<double> jointVelocities2;
    sensor_msgs::JointState jointStates2;
    for (int i=0; i<arm2.nJoints; i++) {
        jointStates2.position.push_back(0.0);
        jointStates2.velocity.push_back(0.0);
    }


    while(ros::ok()) {
        endeffectorVelocity1 = armController1.controlLoop();
        jointVelocities1 = arm1.ikVelocitySolver(endeffectorVelocity1);

        endeffectorVelocity2 = armController2.controlLoop();
        jointVelocities2 = arm2.ikVelocitySolver(endeffectorVelocity1);

        for(int i=0; i<arm1.nJoints; i++){
            jointStates1.velocity[i] = jointVelocities1[i];
            jointStates1.position[i] = arm1.jointArray(i);

            jointStates2.velocity[i] = jointVelocities2[i];
            jointStates2.position[i] = arm2.jointArray(i);
        }


        armPub1.publish(jointStates1);
        armPub2.publish(jointStates2);

        for(int i=0; i<armController1.rvizObstacles.size(); i++){
            markersPub.publish(armController1.rvizObstacles[i]->marker);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}