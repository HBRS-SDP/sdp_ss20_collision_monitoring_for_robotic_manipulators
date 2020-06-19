// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"


class MarkerPublisher {
    public:
        visualization_msgs::Marker marker;
        ros::Publisher ownPublisher;

        MarkerPublisher(ros::Publisher ownPublisher, int type, std::string frame_id, std::string ns, int id, double positionX, double positionY, double positionZ) {
            std::cout << "Hello World!";
            this->ownPublisher = ownPublisher;
            this->marker.type = type;
            this->marker.action = visualization_msgs::Marker::ADD;

            this->marker.header.frame_id = frame_id;
            this->marker.ns = ns;
            this->marker.id = id;

            this->marker.pose.orientation.x = 0.0;
            this->marker.pose.orientation.y = 0.0;
            this->marker.pose.orientation.z = 0.0;
            this->marker.pose.orientation.w = 1.0;

            this->marker.color.a = 1.0; 
            this->marker.color.r = (float) rand() / RAND_MAX;
            this->marker.color.g = (float) rand() / RAND_MAX;
            this->marker.color.b = (float) rand() / RAND_MAX;
        }

        void setRadius(double radius){
            switch (this->marker.type)
            {
            case visualization_msgs::Marker::SPHERE:
                this->marker.scale.x = 2 * radius;
                this->marker.scale.y = 2 * radius;
                this->marker.scale.z = 2 * radius;
                break;
            case visualization_msgs::Marker::CYLINDER:
                this->marker.scale.x = 2 * radius;
                this->marker.scale.y = 2 * radius;
                break;
            default:
                break;
            }
        }

        void setLength(double length){
            switch (this->marker.type)
            {
            case visualization_msgs::Marker::SPHERE:
                break;
            case visualization_msgs::Marker::CYLINDER:
                this->marker.scale.z = length;
                break;
            default:
                break;
            }
        }

        void Publish(){
            this->marker.header.stamp = ros::Time();
            ownPublisher.publish(this->marker);
        }
};

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
        double length, radius;
        int shape;

        std::string frame_id, ns;
        int id;
        double positionX, positionY, positionZ;
        
        
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
                std::cout << "Choose an obstacle type; sphere(1), cylinder (2), capsule(3):\n";
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
                        std::cout<< "Radius: ";
                        std::cin >> radius;
                        marker.scale.x = 2 * radius;
                        marker.scale.y = 2 * radius;
                        marker.scale.z = 2 * radius;

                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;

                        marker.color.a = 1.0; 
                        marker.color.r = (float) rand() / RAND_MAX;
                        marker.color.g = (float) rand() / RAND_MAX;
                        marker.color.b = (float) rand() / RAND_MAX;

                        obstaclePub.publish(marker);

                        break;
                    
                    case 2:
                        std::cout << "Cylinder selected, please input the required values.\n";
                        std::cout << "Frame_id: ";
                        std::cin >> marker.header.frame_id;
                        std::cout << "NameSpace: ";
                        std::cin >> ns;
                        std::cout << "ID: ";
                        std::cin >> id;
                        marker.type = visualization_msgs::Marker::CYLINDER;
                        std::cout << "Input position:\n";
                        std::cout << "\tx: ";
                        std::cin >> marker.pose.position.x;
                        std::cout << "\ty: ";
                        std::cin >> marker.pose.position.y;
                        std::cout << "\tz: ";
                        std::cin >> marker.pose.position.z;
                        std::cout<< "Radius: ";
                        std::cin >> radius;
                        std::cout<< "Length: ";
                        std::cin >> length;
                        marker.scale.x = 2 * radius;
                        marker.scale.y = 2 * radius;
                        marker.scale.z = length;

                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;

                        marker.color.a = 1.0; 
                        marker.color.r = (float) rand() / RAND_MAX;
                        marker.color.g = (float) rand() / RAND_MAX;
                        marker.color.b = (float) rand() / RAND_MAX;

                        obstaclePub.publish(marker);

                        break;
                    case 3:
                    {
                        std::cout << "Capsule selected, please input the required values.\n";
                        std::cout << "Frame_id: ";
                        std::cin >> frame_id;
                        std::cout << "NameSpace: ";
                        std::cin >> ns;
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

                        MarkerPublisher m(obstaclePub, visualization_msgs::Marker::CYLINDER, frame_id, ns, id, positionX, positionY, positionZ);
                        m.setRadius(radius);
                        m.setLength(length);

                        m.Publish();
                        break;
                    }
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