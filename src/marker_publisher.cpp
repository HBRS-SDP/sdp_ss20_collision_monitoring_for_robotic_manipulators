#include "marker_publisher.h"

MarkerPublisher::MarkerPublisher(ros::Publisher ownPublisher, int type, std::string frame_id, std::string ns, int id, double positionX, double positionY, double positionZ, double r, double g, double b, double alpha) {
    std::cout << "0" << std::endl;
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

    this->marker.pose.position.x = positionX;
    this->marker.pose.position.y = positionY;
    this->marker.pose.position.z = positionZ;

    this->marker.color.a = alpha; 
    this->marker.color.r = r;
    this->marker.color.g = g;
    this->marker.color.b = b;
    std::cout << "1" << std::endl;
}

void MarkerPublisher::setRadius(double radius){
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
    case visualization_msgs::Marker::ARROW:
        this->marker.scale.y = 2 * radius;
        this->marker.scale.z = 2 * radius;
        break;
    default:
        break;
    }
}

void MarkerPublisher::setLength(double length){
    switch (this->marker.type)
    {
    case visualization_msgs::Marker::SPHERE:
        break;
    case visualization_msgs::Marker::CYLINDER:
        this->marker.scale.z = length;
        break;
    case visualization_msgs::Marker::ARROW:
        this->marker.scale.x = length;
        break;
    default:
        break;
    }
}

void MarkerPublisher::setPosition(double positionX, double positionY, double positionZ){
    this->marker.pose.position.x = positionX;
    this->marker.pose.position.y = positionY;
    this->marker.pose.position.z = positionZ;
}

void MarkerPublisher::setOrientation(double orientationX, double orientationY, double orientationZ, double orientationW){
    this->marker.pose.orientation.x = orientationX;
    this->marker.pose.orientation.y = orientationY;
    this->marker.pose.orientation.z = orientationZ;
    this->marker.pose.orientation.w = orientationW;
}

void MarkerPublisher::setPoints(Eigen::Vector3d start, Eigen::Vector3d end){
    geometry_msgs::Point startPoint;
    geometry_msgs::Point endPoint;

    switch (this->marker.type)
    {
    case visualization_msgs::Marker::ARROW:
        startPoint.x = start(0);
        startPoint.y = start(1);
        startPoint.z = start(2);

        endPoint.x = end(0);
        endPoint.y = end(1);
        endPoint.z = end(2);

        this->marker.points.clear();
        this->marker.points.push_back(startPoint);
        this->marker.points.push_back(endPoint);
        break;
    
    default:
        break;
    }
    
}

void MarkerPublisher::Publish(){
    this->marker.header.stamp = ros::Time();
    ownPublisher.publish(this->marker);
}
