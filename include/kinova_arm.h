#ifndef KINOVA_ARM_H
#define KINOVA_ARM_H

#define DEBUG

#include <vector>
#include <math.h>
#include <Eigen/Core>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include "primitives.h"
#include "arm.h"



class KinovaArm: public Arm
{
    private:
        KDL::Tree arm_tree;
        int nr_joints;
        std::vector<KDL::Frame*> poses;
        KDL::Chain chain;
        KDL::ChainFkSolverPos_recursive* fksolver;
        KDL::JntArray jointpositions;

    public:
        //Constructor
        KinovaArm(std::string urdf_filename);
        //Destructor
        ~KinovaArm();

        bool updatePose(std::vector<double> joint_positions);
        std::vector<double> base_position;
        std::vector<Primitive> links;
};

#endif // KINOVA_ARM_H