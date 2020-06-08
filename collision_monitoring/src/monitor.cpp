#include "monitor.h"
#include <vector>

Monitor::Monitor(Arm arm, std::vector<Primitive*> obstacles){
    std::cout << arm.links.size() << std::endl;
    this->arm = arm;
    this->obstacles = obstacles;
}

Monitor::~Monitor(){

}

std::vector<std::vector<double>> Monitor::monitorCollisionWithObjects(){
    std::vector<std::vector<double>> distances_to_objects;


    std::cout << "links: " << this->arm.links.size() << std::endl;
    for (int i = 0; i < this->arm.links.size(); i++ ) {
        std::vector<double> distances;
        for ( int j = 0; i < this->obstacles.size(); j++) {
            double dis = this->arm.links[i]->getShortestDistance(this->obstacles[j]);
            distances.push_back(dis);
            std::cout << "value: " << dis << std::endl;
        }
        distances_to_objects.push_back(distances);
    }

    for (auto &i : distances_to_objects) {
        std::cout << "VVVVV";
        for (auto &j : i) {
            std::cout << j << " ";
        }
        std::cout << std::endl;
    }

    return distances_to_objects;
}

std::vector<std::vector<double>> Monitor::monitorCollisionWithArm(){
    std::vector<std::vector<double>> distances_to_objects;
    for (int i = 0; i < this->arm.links.size(); i++) {
        std::vector<double> distances;
        for (int j = 0; j < this->arm.links.size(); j ++) {
            if (i != j) {
                distances.push_back(this->arm.links[i]->getShortestDistance(this->obstacles[j]));
            }
        }
        distances_to_objects.push_back(distances);
    }
    return distances_to_objects;
}