#ifndef MARKERPUBLISHER_H
#define MARKERPUBLISHER_H

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "Eigen/Geometry"
#include "Eigen/Dense"

class MarkerPublisher {
    public:
        visualization_msgs::Marker marker;
        ros::Publisher ownPublisher;

        MarkerPublisher(ros::Publisher ownPublisher, int type, std::string frame_id, 
                        std::string ns, int id, double positionX, double positionY, double positionZ, 
                        double r, double g, double b, double alpha);

        void setRadius(double radius);

        void setLength(double length);

        void setPosition(double positionX, double positionY, double positionZ);

        void setOrientation(double orientationX, double orientationY, double orientationZ, double orientationW);

        void setPoints(Eigen::Vector3d start, Eigen::Vector3d end);

        void Publish();
};

#endif