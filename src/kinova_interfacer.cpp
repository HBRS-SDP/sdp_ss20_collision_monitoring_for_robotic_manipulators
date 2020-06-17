// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"


int main(int argc, char **argv)
{
    // Init ROS listener
    ros::init(argc, argv, "kinova_interfacer");
    ros::NodeHandle n;

    ros::Publisher goalPub = n.advertise<geometry_msgs::Point>("kinova_controller/goal", 1000);
    ros::Publisher obstaclePub = n.advertise<visualization_msgs::Marker>("kinova_controller/obstacles", 1000);
    
    ros::Rate loop_rate(10);

    visualization_msgs::Marker marker;
    geometry_msgs::Point point;


    while(ros::ok()) {
        int mode;
        double diameter;
        int shape;
        std::cout << "Choose input mode; goal(1) or obstacle(2): ";
        std::cin >> mode;
        switch(mode) {
            case 1:
                std::cout << "Goal mode selected please input the new goal position.\n";
                std::cout << "x: ";
                std::cin >> point.x;
                std::cout << "y: ";
                std::cin >> point.y;
                std::cout << "z: ";
                std::cin >> point.z;
                goalPub.publish(point);
                break;

            case 2:
                std::cout << "Choose an obstacle type; sphere(1):\n";
                std::cin >> shape;
                switch(shape) {
                    case 1:
                        std::cout << "Sphere selected, please input the required values.\n";
                        std::cout << "Frame_id: ";
                        std::cin >> marker.header.frame_id;
                        marker.header.stamp = ros::Time();
                        std::cout << "NameSpace: ";
                        std::cin >> marker.ns;
                        std::cout << "ID: ";
                        std::cin >> marker.id;
                        marker.type = visualization_msgs::Marker::SPHERE;
                        marker.action = visualization_msgs::Marker::ADD;
                        std::cout << "Input position:\n";
                        std::cout << "\tx: ";
                        std::cin >> marker.pose.position.x;
                        std::cout << "\ty: ";
                        std::cin >> marker.pose.position.y;
                        std::cout << "\tz: ";
                        std::cin >> marker.pose.position.z;
                        std::cout<< "Diameter: ";
                        std::cin >> diameter;
                        marker.scale.x = diameter;
                        marker.scale.y = diameter;
                        marker.scale.z = diameter;

                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;

                        marker.color.a = 1.0; 
                        marker.color.r = 0.0;
                        marker.color.g = 1.0;
                        marker.color.b = 0.0;

                        obstaclePub.publish(marker);

                        break;

                    default:
                        std::cout << "Incorrect input, please try again.\n";
                    
                }
                break;
                
            default:
                    std::cout << "Incorrect input, please try again.\n";
                    

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}