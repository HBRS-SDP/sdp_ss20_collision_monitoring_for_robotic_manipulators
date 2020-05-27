#include <Eigen/Dense>
#include "obstacle.h"
#include "primitive.h"


// Constructor
Obstacle::Obstacle(Eigen::Matrix4d pose, std::Vector<Primitive> primitives){
    this->pose = pose;
    this->primitves = primitives;
}

// Destructure
Obstacle::~Obstacle(){

}

void Obstacle::updatePose(Eigen::Matrix4d pose){
    this->pose = pose;
}