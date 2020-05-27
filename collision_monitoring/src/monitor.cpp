#include "monitor.h"

Monitor::Monitor(Arm arm, std::vector<Primitive> obstacles){
    //this->arm = arm;
    //this->obstacles = obstacles;
}

Monitor::~Monitor(){

}

void Monitor::monitorCollisionWithObjects(){
    // to implement
    // for each link of arm
        // for each obstacle
            // for each primitive in obstacle
                //calculate distance to link
}

void Monitor::monitorCollisionWithArm(){
    // to implement
    //for each link of arm
        //calculate distance to other link
}