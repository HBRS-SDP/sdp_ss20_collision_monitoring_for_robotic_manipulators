#ifndef KINOVA_ARM_H
#define KINOVA_ARM_H

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "primitives.h"
#include "arm.h"



class KinovaArm: public Arm
{
    private:
        KDL::Tree arm_tree;
        int nr_joints;
        KDL::Frame pos;
        KDL::Chain chain;

    public:
        //Constructor
        KinovaArm(std::string urdf_filename);
        //Destructor
        virtual ~KinovaArm();

        virtual bool updatePose(std::vector<double>);
        std::vector<double> base_position;
        std::vector<Primitive> links;
};

#endif // KINOVA_ARM_H