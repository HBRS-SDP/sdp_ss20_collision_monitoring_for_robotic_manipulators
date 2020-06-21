#ifndef ARM_H
#define ARM_H


#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "primitives.h"


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

        virtual Eigen::Matrix4d  getPose(void) = 0;

        virtual Eigen::Matrix4d  getPose(int jointNumber) = 0;

        Eigen::Matrix4d baseTransform;
        std::vector<Primitive*> links;
        /// The number of joints in the chain
        int nJoints;

};


#endif // ARM_H