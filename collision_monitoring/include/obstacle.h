#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>
#include <Eigen/Dense>
#include "primitives.h"

class Obstacle {

    protected: 

        Eigen::Matrix4d pose;
        std::vector<Primitive> primitives;
    
    public: 

        void updatePosition(Eigen::Matrix4d pose);
        ~Obstacle();
};

#endif // OBSTACLE_H