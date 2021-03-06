#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

// Standard libraries
#include <vector>
#include <math.h>
#include <memory>
#include <iostream>

// Eigen libraries
#include <Eigen/Core>
#include <Eigen/Geometry>


//ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

//Collision monitoring imports
#include "primitives.h"
#include "kinova_arm.h"
#include "monitor.h"
#include "marker_publisher.h"
// #include "arm_controller.h"

//Math Libraries
#include <math.h> 

/**
 * A class for dealing with obstacle displaying in ROS.
 */
class RvizObstacle
{
    public:
        /// The index of the obstacle in Monitor obstacle list
        int idx;

        /// The marker that represents the obstacle in rviz
        visualization_msgs::Marker marker;

        /// The pose of the marker
        Eigen::Matrix4d pose;

        /** 
         * Constructor
         * 
         * @param markerIn The marker with dimensions suiting the obstacle
         * @param index The location of the obstacle in the monitor obstacle list
         */
        RvizObstacle(visualization_msgs::Marker::ConstPtr markerIn, int index);

        /// Destructor
        ~RvizObstacle();

        /**
         * Function for updating the position of the obstacle
         * 
         * @param markerIn The marker representing the new position of the obstacle
         * @return The new Pose of the obstacle
         */
        Eigen::Matrix4d updatePose(visualization_msgs::Marker::ConstPtr markerIn);

};


/**
 * Arm controller class to demonstate the use of the Collision monitoring library.
 * 
 * The class uses the monitor class from the custom library to control a 7DOF
 * kinova arm implementing the paper: [1]H. Hoffmann, P. Pastor, D.-H. Park, and 
 * S. Schaal, “Biologically-inspired dynamical systems for movement generation: 
 * Automatic real-time goal adaptation and obstacle avoidance,”
 * in 2009 IEEE International Conference on Robotics and Automation, 
 * Kobe, May 2009, pp. 2587–2592, doi: 10.1109/ROBOT.2009.5152423.
 */
class BaseController
{
    public:
        /**
         * Base Controller constructor
         * 
         * @param monitorObject The monitor instance used to get the distance
         *     from the base to any obstacles
         * @param k Costant for the obstacle avoidance algorithim
         * @param d Costant for the obstacle avoidance algorithim
         * @param gamma Costant for the obstacle avoidance algorithim
         * @param beta Costant for the obstacle avoidance algorithim
         * @return An instance of CollisionMonitor class
         */
        BaseController(Monitor* monitorObject, double k, double d, 
                                            double gamma, double beta);
        /// Base Destructor
        ~BaseController();

        /// The monitor class used to perform collision monitoring
        Monitor* monitor;

        /// The goal point of the robot motion
        Eigen::Vector3d goal;

        /**
         * Callback function updating the base positions
         * 
         * @param msg The ros sensor messsage containing the odometry messages
         */
        void baseCallback(const nav_msgs::Odometry::ConstPtr &msg);      

        /**
         * Callback function updating goal position of the arms endeffector
         * 
         * @param msg The ros sensor messsage containing the goal position
         */
        void goalCallback(const geometry_msgs::Point::ConstPtr& msg);

        /**
         * Function that uses internal parameters to send instructions to arm
         * 
         * This is where the obstacle avoidance is implemented and is based off 
         * of the paper: 
         */
        
	    geometry_msgs::Twist control_loop(void);

        /**
        * Creates the obstacle potential field value used in the control loop [1]
        * 
        * @param currentPosition The current position of the endeffector
        * @param velocity The current velocity of the endeffector
        * @return The potential feild vector produced from the obstacles
        */ 
        Eigen::Vector3d obstaclePotentialField(Eigen::Vector3d currentPosition, 
                                                Eigen::Vector3d velocity);

        /**
         * A callback function that updates or adds obstacles
         * 
         * @param msg the ros marker message with the object parameters
         */
        void updateObstacles(const visualization_msgs::Marker::ConstPtr& msg);

        /// A list of obstacles that are displayed in rviz
        std::vector<RvizObstacle*> rvizObstacles;

    private:

        /// The number of joints in the arm
        int numJoints;

        /// The constants used in the obstacle avoidance algorithm
        double D;
        double K;
        double gamma;
        double beta;

        // Lists of parameters and objects passed between functions
        Eigen::Vector3d basePosition;  // x,y,theta
        std::vector<double> objectDistances;
        std::vector<std::vector<double>> armDistances;
        Eigen::Vector4d origin;
        std::vector<Primitive*> obstaclesAllocated;
        geometry_msgs::Twist speed;
        ros::NodeHandle n;
        ros::Publisher CubePub,arrowsPub_base;

};

#endif // BASE_CONTROLLER_H
