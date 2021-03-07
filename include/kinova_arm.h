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
#include <kdl/chainiksolvervel_wdls.hpp>
#include "primitives.h"
#include "arm.h"


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

        /**
         * KinovaArm constructor with set baseposition
         * 
         * @param urdf_filename The global location of the urdf file used to
         *     import the arm kinematics model
         * @param inputBaseTransform The global position of the robot base
         * @return An instance of KinovaArm class
         */
        KinovaArm(std::string urdf_filename, Eigen::Matrix4d inputBaseTransform);
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

        /**
         * A function to find the inverse kinematics of an endeffector trajectory
         * 
         * @param twist The KDL::Twist velocity vector
         * @return A vector of joint velocities used to achieve the desired velocity
         */
        std::vector<double> ikVelocitySolver(KDL::Twist twist);

        /**
         * A function to find the final joint pose
         * 
         * @return The last joint pose
         */
        Eigen::Matrix4d  getPose(void);

        /**
         * A function to find the joint pose of a given joint
         * 
         * @param frameNumber The joint number to solve for the pose of
         * @return The last joint pose
         */
        Eigen::Matrix4d  getPose(int frameNumber);

        /// The KDL joint array to hold the joint angles
        KDL::JntArray jointArray;
        /// The KDL joint array to hold the joint velocities
        KDL::JntArray jointVels;



    private:

        /// A vector of the length of each of the links
        std::vector<double> lengths;

        /// A vector of the radius of each of the links
        std::vector<double> radii;

        ///  A vector of all the link KDL frames
        std::vector<KDL::Frame*> localPoses;

        /// The KDL chain used for calculating kinematics
        KDL::Chain fkChain;

        /// Mathematical constants, declared in constructor for speed
        Eigen::Vector4d origin;
        Eigen::Vector3d directionVect;
        Eigen::Matrix3d i3;

        /**
         * Transforms local KDL::Frames to global Eigen::Matrix4d
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





class NarkinBase: public Base
{
    public:

        /**
         * NarkinBase constructor with set baseposition
         * 
         * @param inputBaseTransform The global position of the robot base
         * @return An instance of NarkinBase class
         */
        NarkinBase( Eigen::Vector3d inputBaseTransform);
        /// KinovaArm Destructor
        ~NarkinBase();

        /**
         * A function to update the current virtual representation of the base
         * 
         * @param basePositions (center) The 3D pose of robot base.
         * @return The boolean true for a successful update, False otherwise
         */
        bool updatePose(Eigen::Vector3d basePositions);

        

        /**
         * A function to find the final base pose
         * 
         * @return The last base pose
         */
        Eigen::Vector3d getPose(void);

        // Eigen::Matrix4d  getPose(int frameNumber);

        /**
         * A function to find the joint pose of a given joint
         * 
         * @param frameNumber The joint number to solve for the pose of
         * @return The last joint pose
         */
        

    private:


        /// Mathematical constants, declared in constructor for speed
        Eigen::Vector3d center;
        Eigen::Vector3d half_dimensions;
        Eigen::Matrix3d i3;
        KDL::Frame* localPose;

        
};

#endif // KINOVA_ARM_H