#ifndef ARM_H
#define ARM_H


#include <vector>
#include "primitives.h"


class Arm
{
    protected:
        //Destructor
        virtual ~Arm();

        virtual bool updatePose(std::vector<double>);
        std::vector<double> base_position;
        std::vector<Primitive> links;
};




#endif // ARM_H