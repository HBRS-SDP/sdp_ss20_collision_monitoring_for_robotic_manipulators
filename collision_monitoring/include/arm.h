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
        virtual ~Arm();

};




#endif // ARM_H