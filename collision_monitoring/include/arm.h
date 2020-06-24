#ifndef ARM_H
#define ARM_H


#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "primitives.h"

/**
 * A pure virtual class that represents a robotic manipulator
 * 
 * This class is an interface class and so the developer is to implement
 * this class to suit the type of robotic manipulator used
 */
class Arm
{
    public:
    
        //Destructor
        virtual ~Arm();

        /**
         * A function to update the current virtual representation of the arm
         * 
         * @param jointPositions The angular positions of the arm joints
         *     in order of the joint in radians
         * @return The boolean true for a successful update, False otherwise
         */
        virtual bool updatePose(std::vector<double>);

        /// Used to get the endeffector frame of the manipulator
        virtual Eigen::Matrix4d getPose(void) = 0;
        /// Used to get the frame 
        virtual Eigen::Matrix4d getPose(int frameNumber) = 0;

        /// The homogeneous transformation from the world to arm base frame
        Eigen::Matrix4d baseTransform;

        /// The list of link primitives that represent the manipulator
        std::vector<Primitive*> links;
        /// The number of links in the chain
        int nLinks;
        /// The number of joints in the chain
        int nJoints;
        /// The number of frames in the arm
        int nFrames;

};


#endif // ARM_H