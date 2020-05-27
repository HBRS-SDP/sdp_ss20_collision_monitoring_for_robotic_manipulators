#ifndef MONITOR_H
#define MONITOR_H

#include <vector>
#include "arm.h"
#include "primitives.h"

class Monitor
{

    public: 

        Arm arm;
        std::vector<Primitive> obstacles;
        // Void on return type of these functions temporally
        void monitorCollisionWithObjects();
        void monitorCollisionWithArm();
        Monitor(Arm arm, std::vector<Primitive> obstacles);
        ~Monitor();
};

#endif // MONITOR_H