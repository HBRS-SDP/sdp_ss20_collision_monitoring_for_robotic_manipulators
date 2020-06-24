// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include "marker_publisher.h"

int main(int argc, char **argv)
{
    // Init ROS listener
    ros::init(argc, argv, "kinova_interfacer");
    ros::NodeHandle n;
    std::string goalTopic;

    n.param<std::string>(ros::this_node::getName()+"/goal_topic", goalTopic, ros::this_node::getName()+"/goal");

    ros::Publisher goalPub = n.advertise<geometry_msgs::Point>(goalTopic, 1000);
    ros::Publisher obstaclePub = n.advertise<visualization_msgs::Marker>("kinova_controller/obstacles", 1000);
    
    ros::Rate loop_rate(10);

    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    bool exit = false;

    Eigen::Vector3d startArrow(0.1, 0, 0); 
    Eigen::Vector3d endArrow(0.1, 0, 0.5);


    while(ros::ok() && !exit) {
        int mode;
        double length, radius, r, g, b;
        int shape;

        std::string frame_id, ns;
        int id;
        double positionX, positionY, positionZ;
        
        
        std::cout << "Choose input mode; goal(1), obstacle(2) or exit (0): ";
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
                std::cout << "Choose an obstacle type; sphere(1), cylinder (2), capsule(3):\n";
                std::cin >> shape;
                switch(shape) {
                    case 1:
                    {
                        std::cout << "Sphere selected, please input the required values.\n";
                        std::cout << "ID: ";
                        std::cin >> id;
                        std::cout << "Input position:\n";
                        std::cout << "\tx: ";
                        std::cin >> positionX;
                        std::cout << "\ty: ";
                        std::cin >> positionY;
                        std::cout << "\tz: ";
                        std::cin >> positionZ;
                        std::cout<< "Radius: ";
                        std::cin >> radius;

                        r = (float) rand() / RAND_MAX;
                        g = (float) rand() / RAND_MAX;
                        b = (float) rand() / RAND_MAX;

                        MarkerPublisher mPublisher(obstaclePub, visualization_msgs::Marker::SPHERE, "base_link", "obstacles", id, positionX, positionY, positionZ, r, g, b, 0.5);
                        mPublisher.setRadius(radius);

                        // MarkerPublisher mPublisherArrow(obstaclePub, visualization_msgs::Marker::ARROW, frame_id, ns, id + 1, positionX, positionY, positionZ, g, b, r, 1.0);
                        // mPublisherArrow.setRadius(radius / 5);
                        // mPublisherArrow.setLength(radius * 10);

                        mPublisher.Publish();
                        // mPublisherArrow.Publish();
                        
                        break;
                    }
                    
                    case 2:
                    {
                        std::cout << "Cylinder selected, please input the required values.\n";
                        std::cout << "ID: ";
                        std::cin >> id;
                        std::cout << "Input position:\n";
                        std::cout << "\tx: ";
                        std::cin >> positionX;
                        std::cout << "\ty: ";
                        std::cin >> positionY;
                        std::cout << "\tz: ";
                        std::cin >> positionZ;
                        std::cout<< "Radius: ";
                        std::cin >> radius;
                        std::cout<< "Length: ";
                        std::cin >> length;

                        r = (float) rand() / RAND_MAX;
                        g = (float) rand() / RAND_MAX;
                        b = (float) rand() / RAND_MAX;

                        MarkerPublisher mPublisher(obstaclePub, visualization_msgs::Marker::CYLINDER, "base_link", "obstacles", id, positionX, positionY, positionZ, r, g, b, 1.0);
                        mPublisher.setRadius(radius);
                        mPublisher.setLength(length);

                        mPublisher.Publish();
                        break;
                    }
                    case 3:
                    {
                        std::cout << "Capsule selected, please input the required values.\n";
                        std::cout << "ID: ";
                        std::cin >> id;
                        std::cout << "Input position:\n";
                        std::cout << "\tx: ";
                        std::cin >> positionX;
                        std::cout << "\ty: ";
                        std::cin >> positionY;
                        std::cout << "\tz: ";
                        std::cin >> positionZ;
                        std::cout<< "Radius: ";
                        std::cin >> radius;
                        std::cout<< "Length: ";
                        std::cin >> length;

                        r = (float) rand() / RAND_MAX;
                        g = (float) rand() / RAND_MAX;
                        b = (float) rand() / RAND_MAX;

                        MarkerPublisher mPublisher(obstaclePub, visualization_msgs::Marker::CYLINDER, "base_link", "obstacles", id, positionX, positionY, positionZ, r, g, b, 1.0);
                        mPublisher.setRadius(radius);
                        mPublisher.setLength(length);

                        Eigen::Vector3d base(positionX, positionY, positionZ - length / 2);
                        Eigen::Vector3d end(positionX, positionY, positionZ + length / 2);
                        Eigen::Quaterniond quat(mPublisher.marker.pose.orientation.w, mPublisher.marker.pose.orientation.x, mPublisher.marker.pose.orientation.y, mPublisher.marker.pose.orientation.z);

                        Eigen::Vector3d sphereBase = quat * base;
                        Eigen::Vector3d sphereEnd = quat * end;

                        MarkerPublisher mPublisherBase(obstaclePub, visualization_msgs::Marker::SPHERE, frame_id, ns, id + 1, sphereBase[0], sphereBase[1], sphereBase[2], r, g, b, 1.0);
                        mPublisherBase.setRadius(radius);

                        MarkerPublisher mPublisherEnd(obstaclePub, visualization_msgs::Marker::SPHERE, frame_id, ns, id + 2, sphereEnd[0], sphereEnd[1], sphereEnd[2], r, g, b, 1.0);
                        mPublisherEnd.setRadius(radius);

                        mPublisher.Publish();
                        mPublisherBase.Publish();
                        mPublisherEnd.Publish();
                        break;
                    }
                    default:
                        std::cout << "Incorrect input, please try again.\n";
                    
                }
                break;
            case 0:
                exit = true;
                break;
            default:
                    std::cout << "Incorrect input, please try again.\n";
                    

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}