#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include <Eigen/Dense>
#include "primitives.h"

class Obstacle
{

    protected: 

        Eigen::Matrix4d pose;
        std::vector<Primitive*> primitives;
    
    public: 

        void updatePose(Eigen::Matrix4d pose);
        Obstacle(Eigen::Matrix4d pose, std::vector<Primitive*> primitives);
        ~Obstacle();
};

#endif // OBSTACLE_H