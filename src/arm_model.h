#ifndef ARM_MODEL_H
#define ARM_MODEL_H

// KDL Libraries
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <vector>

#include "basic_shapes.h"


class KinovaArm
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
        ~KinovaArm();

        bool updatePose(std::vector<double>);
        std::vector<Link> links;
}

class Link: public shapes::Cylinder
{
    private:
        //any private variables
    public:
        Link* parent_link;
        std::vector<double> base_point;
        std::vector<double> endpoint;

}

#endif // ARM_MODEL_H