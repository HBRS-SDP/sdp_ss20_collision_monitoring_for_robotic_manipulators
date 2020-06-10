#ifndef MONITOR_H
#define MONITOR_H

#include <vector>
#include <iostream>
#include "arm.h"
#include "primitives.h"

/**
 * A collision monitor with methods to determine the distance to obstacles or links of the arm
 * 
 * The Monitor class is a collision monitor which stores the address of an instance of the class Arm
 * and maintains a list of obstacles. It can perform collision monitoring with obtacles by determining
 * the distance to obstacles in the workspace, or collision monitoring with the arm itself by 
 * monitoring the distance between the links.  
 */
class Monitor
{

    public: 

        Arm* arm;
        std::vector<Primitive*> obstacles;

        /** monitor collision with obstacles. 
        *
        */
        std::vector<std::vector<double>> monitorCollisionWithObjects();
        std::vector<std::vector<double>> monitorCollisionWithArm();
        void addObstacle(Primitive* obstacle);
        void addObstacle(Arm* arm);
        Monitor(Arm* arm);
        ~Monitor();
};

#endif // MONITOR_H