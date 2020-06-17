#ifndef MONITOR_H
#define MONITOR_H

#include <vector>
#include <iostream>
#include "arm.h"
#include "primitives.h"

/**
 * A collision monitor to determine the distance to obstacles and other links
 * 
 * The Monitor class is a collision monitor which stores the address of an 
 * instance of the class Arm and maintains a list of obstacles. It can perform 
 * collision monitoring with obstacles by determining the distance to obstacles 
 * in the workspace, or collision monitoring with the arm itself by monitoring 
 * the distance between the links.  
 */
class Monitor
{

    public: 


        Arm* arm; /* Arm to monitor */
        std::vector<Primitive*> obstacles; /* Obstacles in the workspace */
        std::vector<Primitive*> obstaclesToDelete; /* Obstacles to delete in 
        destructor */
        /** Collision monitoring with obstacles. 
        *
        * This methods monitors the distance from one link of the arm 
        * to obstacles in the workspace. 
        *
        * @returns a matrix with the distance of each link to the other 
        * obstacles.
        */
        std::vector<std::vector<double>> distanceToObjects();

        /** Collision monitoring with the arm itself.
        *
        * This methods monitors the distance from one link of the arm 
        * to other links. 
        *
        * @returns a matrix with the distance of each link to the other links.
        */
        std::vector<std::vector<double>> distanceBetweenArmLinks();

        /** Adds primitive to list of obstacles
        *
        * Adds a primitive to the list of obstacles.
        * @param obstacle address of the obstacle to be added.
        */
        void addObstacle(Primitive* obstacle);
        void addObstacle(Sphere* obstacle);
        void addObstacle(Capsule* obstacle);
        /** Adds arm to list of obstacles
        *
        * This method decomposes an arm into its primitives to add it into 
        * the vector of obstacles.
        *
        * @param arm address of the arm to be treated as an obstacle.
        */
        void addObstacle(Arm* arm);

        /** Constructor of Monitor
        *
        * This is the constructor for the monitor class, it takes as 
        * paramter the arm to be monitored. 
        *
        * @param arm arm to monitor collision.
        */
        Monitor(Arm* arm);

        /** Destructor for the monitor class
        */
        ~Monitor();
};

#endif // MONITOR_H