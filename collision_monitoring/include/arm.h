#ifndef ARM_H
#define ARM_H


#include <vector>
#include "primitives.h"


class Arm
{

    protected:

        virtual bool updatePose(std::vector<double>);
        std::vector<double> base_position;
        std::vector<Primitive> links;

    public:
    
        //Destructor
        ~Arm() {}

};

class SampleArm
{
    public:
        SampleArm();
        //Destructor
        virtual ~SampleArm();

    protected:

        virtual bool updatePose(std::vector<double>);
        std::vector<double> base_position;
        std::vector<Primitive> links;

};






#endif // ARM_H