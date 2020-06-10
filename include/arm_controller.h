#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <iostream>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include "primitives.h"
#include "kinova_arm.h"
#include "monitor.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"


/**
 * An implementation of the Arm interface.
 * 
 * This implementation is specifically for the KINOVA arm in the HBRS robotics
 * lab.
 */

class ArmController
{
    public:
        /**
         * KinovaArm constructor
         * 
         * @param monitorObject The monitor instance used to get the distance
         *     from the arm to any obstacles
         * @return An instance of CollisionMonitor class
         */
        ArmController(Monitor* monitorObject);
        /// KinovaArm Destructor
        ~ArmController();

        Monitor* monitor;
        Eigen::Vector3d goal;

        /**
         * Callback function updating the arm positions
         * 
         * @param msg The ros sensor messsage containing the joint angles
         */
        void armCallback(const sensor_msgs::JointState::ConstPtr& msg);

        /**
         * Callback function updating goal position of the arms endeffector
         * 
         * @param msg The ros sensor messsage containing the goal position
         */
        void goalCallback(const geometry_msgs::Point::ConstPtr& msg);

    private:

        std::vector<std::vector<double>> objectDistance;
        std::vector<std::vector<double>> armDistances;

};

#endif // ARM_CONTROLLER_H