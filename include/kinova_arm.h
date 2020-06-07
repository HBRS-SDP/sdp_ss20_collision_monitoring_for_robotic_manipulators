#ifndef KINOVA_ARM_H
#define KINOVA_ARM_H

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
#include "arm.h"

// the debug var to turn verbose on and off
#define DEBUG

/**
 * An implementation of the Arm interface.
 * 
 * This implementation is specifically for the KINOVA arm in the HBRS robotics
 * lab.
 */

class KinovaArm: public Arm
{
    public:
        /**
         * KinovaArm constructor
         * 
         * @param urdf_filename The global location of the urdf file used to
         *     import the arm kinematics model
         * @return An instance of KinovaArm class
         */
        KinovaArm(std::string urdf_filename);
        /// KinovaArm Destructor
        ~KinovaArm();

        /**
         * A function to update the current virtual representation of the arm
         * 
         * @param jointPositions The angular positions of the arm joints
         *     in order of the joint in radians
         * @return The boolean true for a successful update, False otherwise
         */
        bool updatePose(std::vector<double> jointPositions);

        /// The homogeneous transform of the base of the arm in the global frame
        std::vector<double> base_position;

        /**
         * The list of links used for distance calculation, these are updated
         * by the updatePose() function. For the kinova arm the links are 
         * modelled by cylinders with a length and radius
         */
        std::vector<std::shared_ptr<Primitive> > links;


    private:

        /// A vector of the length of each of the links
        std::vector<double> lengths;

        /// A vector of the radius of each of the links
        std::vector<double> radii;

        /// This is used to import the URDF file
        KDL::Tree arm_tree;

        /// The number of joints in the chain
        int nJoints;

        ///  A vector of all the link KDL frames
        std::vector<std::shared_ptr<KDL::Frame> > poses;

        /// The KDL chain used for calculating kinematics
        KDL::Chain chain;

        KDL::JntArray jointArray;

        /**
         * Transforms KDL::Frames to Eigen::Matrix4d
         * 
         * A function that is used to transform KDL frames into homogeneous
         * transformation functions in Eigen format
         * 
         * @param frame The KDL::Frame to transform
         * 
         * @return Eigen::Matrix4d representation of a homogeneous transform
         */
        Eigen::Matrix4d frameToMatrix(KDL::Frame frame);

        /**
         * Creates a link pose from the current (start) and next (end) link poses
         * 
         * The function converts the start and end link to Frames to a pose with
         * the position at the start frame and the z axis pointing towards the 
         * end frame.
         * 
         * @param startLink The current or start link used as the base of the 
         *     cylinder
         * @param endlink The next or end link that gives the z direction of the
         *     pose
         * 
         * @return The homogenous representation of the cylinder pose
         */
        Eigen::Matrix4d linkFramesToPose(KDL::Frame startLink, KDL::Frame endLink);

};

#endif // KINOVA_ARM_H