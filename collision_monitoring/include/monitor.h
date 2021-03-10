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

        /// Arm for the monitor
        Arm* arm; 
		Base* base;

        /// Obstacles in the workspace
        std::vector<Primitive*> obstacles; 
        /// Obstacles to delete in destructor
        std::vector<Primitive*> obstaclesToDelete; 
        /** Collision monitoring with obstacles. 
        *
        * This methods monitors the distance from one link of the arm 
        * to obstacles in the workspace. 
        *
        * @returns a matrix with the distance of each link to the other 
        * obstacles.
        */
        std::vector<std::vector<double>> distanceToObjects();
	 /** Collision monitoring with the base and other obstacles.
        *
        * This methods monitors the distance from base of the robot 
        * to other obstacles. 
        *
        * @returns a matrix with the distance of each link to the other links.
        */
        std::vector<double> baseDistanceToObjects();

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
        /** Adds primitive to list of obstacles
        *
        * Adds a primitive to the list of obstacles.
        * @param obstacle address of the obstacle to be added.
        */
        void addObstacle(Sphere* obstacle);
        /** Adds primitive to list of obstacles
        *
        * Adds a primitive to the list of obstacles.
        * @param obstacle address of the obstacle to be added.
        */
        void addObstacle(Capsule* obstacle);
        /** Adds arm to list of obstacles
        *
        * This method decomposes an arm into its primitives to add it into 
        * the vector of obstacles.
        *
        * @param arm address of the arm to be treated as an obstacle.
        */
        void addObstacle(Arm* arm);
	
        /** Adds base to list of obstacles
        *
        * This method decomposes a base into its box3 primitive to add it into 
        * the vector of obstacles.
        *
        * @param base address of the base to be treated as an obstacle.
        */
        void addObstacle(Base* base_obstacle);
	/** Adds box to list of obstacles
         *
        * Adds a box to the list of obstacles.
        * @param box address of the box obstacle to be added.
        */



	void addObstacle(Box3* box); 
        /** Constructor of Monitor
        *
        * This is the constructor for the monitor class, it takes as 
        * paramter the arm to be monitored. 
        *
        * @param arm arm to monitor collision.
        */
  /** Constructor of Monitor
        *
        * This is the constructor for the monitor class, it takes as 
        * paramter the base to be monitored. 
        *
        * @param base base to monitor collision.
        */

        
        Monitor(Base* base);
        Monitor(Arm* arm);

        /** Destructor for the monitor class
        */
        ~Monitor();
};

#endif // MONITOR_H