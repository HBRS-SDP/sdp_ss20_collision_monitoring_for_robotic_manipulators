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


class Arm
{
    protected:
        //Constructor
        virtual Arm();
        //Destructor
        virtual ~Arm();

        virtual bool updatePose(std::vector<double>);
        virtual std::vector<double> base_position;
        virtual std::vector<Primitive> links;
}

class KinovaArm: public Arm
{
    private:
        KDL::Tree arm_tree;
        int nr_joints;
        KDL::Frame pos;
        KDL::Chain chain;

    public:
        //Constructor
        virtual Arm(std::string urdf_filename);
        //Destructor
        virtual ~Arm();

        virtual bool updatePose(std::vector<double>);
        virtual std::vector<double> base_position;
        virtual std::vector<Primitive> links;
}


#endif // ARM_MODEL_H