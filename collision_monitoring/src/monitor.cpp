#include "monitor.h"
#include <vector>

Monitor::Monitor(Arm* arm, std::vector<Primitive*> obstacles){
    this->arm = arm;
    this->obstacles = obstacles;
}

Monitor::~Monitor(){

}

std::vector<std::vector<double>> Monitor::monitorCollisionWithObjects(){
    
    std::vector<std::vector<double>> distances_to_objects;

    for (int i = 0; i < this->obstacles.size(); i++ ) {
        
        std::vector<double> distances;

        for (int j = 0; j < this->arm->links.size(); j++) {
            distances.push_back(this->arm->links[j]->getShortestDistance( this->obstacles[i] ));
        }
        
        distances_to_objects.push_back(distances);
    }

    #ifdef DEBUG
    for (int i = 0; i < distances_to_objects.size(); i++) {
        std::cout << "distances to obstacle [" << i << "]:";

        for (int j = 0; j < distances_to_objects[i].size(); j++){
            std::cout << distances_to_objects[i][j] << " "; 
        }
        std::cout << std::endl;
    }
    #endif //DEBUG

    return distances_to_objects;
}

std::vector<std::vector<double>> Monitor::monitorCollisionWithArm(){
    
    std::vector<std::vector<double>> distances_to_objects;

    for (int i = 0; i < this->arm->links.size(); i++) {
        std::vector<double> distances;
        for (int j = 0; j < this->arm->links.size(); j++) {
            if (i != j) {
                distances.push_back(this->arm->links[i]->getShortestDistance(this->arm->links[j]));
            } else {
                distances.push_back(0);
            }
        }
        distances_to_objects.push_back(distances);
    }

    #ifdef DEBUG
    for (int i = 0; i < distances_to_objects.size(); i++) {
        std::cout << "distances from link [" << i << "] to other links:";

        for (int j = 0; j < distances_to_objects[i].size(); j++){
            std::cout << distances_to_objects[i][j] << " "; 
        }
        std::cout << std::endl;
    }
    #endif //DEBUG

    return distances_to_objects;
}