#include "marker_publisher.h"

MarkerPublisher::MarkerPublisher(ros::Publisher ownPublisher, int type, std::string frame_id, std::string ns, int id, double positionX, double positionY, double positionZ, double r, double g, double b) {
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

    this->marker.color.a = 1.0; 
    this->marker.color.r = r;
    this->marker.color.g = g;
    this->marker.color.b = b;
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

void MarkerPublisher::Publish(){
    this->marker.header.stamp = ros::Time();
    ownPublisher.publish(this->marker);
}
