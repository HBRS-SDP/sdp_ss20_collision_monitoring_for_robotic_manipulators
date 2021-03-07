// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
#include "primitives.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
//#include "arm_controller.h"
#include "base_controller.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "base_controller");
    std::string nodeName = ros::this_node::getName();
    #ifdef DEBUG
        std::cout << "nodeName: " << nodeName << std::endl;
    #endif //DEBUG
    ros::NodeHandle n;
    std::string goalTopic;
    std::string narkoPosition;

    // setup the first monitor function
    n.param<std::string>(ros::this_node::getName()+"/goal_topic", goalTopic, "/goal_point");
    n.param<std::string>(ros::this_node::getName()+"/narko_base_link", narkoPosition, "/narko_base_link");
    double K;
    double D;
    double gamma;
    double beta;
    n.param<double>("/K", K, 0.5);
    n.param<double>("/D", D, 0);
    n.param<double>("/gamma", gamma, 100);
    n.param<double>("/beta", beta, 20/3.1425);

//    Eigen::Matrix4d basePosition;
//    basePosition<< 0,0,0,0.0,
//                   0,0,0,0.0,
//                   0,0,0,0.0,
//                   0,0,0,1;


    Eigen::Vector3d basePosition;

    basePosition[0]=0.0;
    basePosition[1]=0.0;
    basePosition[2]=-1.5;
    NarkinBase base1(basePosition);
    // KinovaArm arm1(model);
    Monitor monitor1(&base1);
    double radius_1 = 30;
    Eigen::Matrix4d pose_1;
    pose_1 << 0.12071974, -0.60161281, -0.78961305, 25,
                -0.60161281,  0.58837018, -0.54026156, 70,
                0.78961305,  0.54026156, -0.29091007, 20,
                0,          0,          0,          1;
    Sphere *sphere_1 = new Sphere(pose_1, radius_1);
    monitor1.addObstacle(sphere_1);
    Eigen::Vector3d initPose;
    initPose[0]=0.0; // x position of center of box
    initPose[1]=0.0; // y position of center of box
    initPose[2]=1.5; // z position of center of box
    base1.updatePose(initPose);

    // Create the armController class based off the first monitor

    BaseController baseController1(&monitor1, K, D, gamma, beta);
    
    // Init ROS listener
    ros::Subscriber goalSub = n.subscribe(goalTopic, 1000, &BaseController::goalCallback, &baseController1);
    ros::Subscriber obstacleSub = n.subscribe("kinova_controller/obstacles", 1000, &BaseController::updateObstacles, &baseController1);
    ros::Subscriber basePosSub = n.subscribe("/odom", 1000, &BaseController::baseCallback, &baseController1);

    ros::Publisher markersPub = n.advertise<visualization_msgs::Marker>("kinova_controller/markers", 1000);
    ros::Publisher baseVelPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::Rate loop_rate(10); 


    
    geometry_msgs::Twist baseVelocity;
    
    while(ros::ok()) {
        baseVelocity = baseController1.control_loop();


        baseVelPub.publish(baseVelocity);
        for(int i=0; i<baseController1.rvizObstacles.size(); i++){
            markersPub.publish(baseController1.rvizObstacles[i]->marker);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}





