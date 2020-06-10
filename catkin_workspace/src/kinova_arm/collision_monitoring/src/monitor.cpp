#include "monitor.h"

Monitor::Monitor(Arm arm, std::vector<Primitive*> obstacles){
    this->arm = arm;
    this->obstacles = obstacles;
}

Monitor::~Monitor(){

}

void Monitor::monitorCollisionWithObjects(){

    for (int i = 0; i < this->arm.links.size(); i++ ) {
        for ( int j = 0; i < this->obstacles.size(); j++) {
            double distance;
            distance = this->arm.links[i]->getShortestDistance(this->obstacles[j]);
        }
    }
}

void Monitor::monitorCollisionWithArm(){
    for (int i = 0; i < this->arm.links.size(); i++) {
        for (int j = 0; j < this->arm.links.size(); j ++) {
            if (i != j) {
                double distance;
                distance = this->arm.links[i]->getShortestDistance(this->arm.links[j]);
            }
        }
    }
}