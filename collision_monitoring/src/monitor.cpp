#include "monitor.h"
#include <vector>

Monitor::Monitor(Arm* arm){
    this->arm = arm;
}

Monitor::~Monitor(){

}

void Monitor::addObstacle(Primitive* obstacle) {
    this->obstacles.push_back(obstacle);
}

void Monitor::addObstacle(Arm* arm) {
    // Adds every link of the arm (a primitve) to the obstacles vector.
    for (int i = 0; i < arm->links.size(); i++) {
        this->obstacles.push_back(arm->links[i]);
    }
}

std::vector<std::vector<double>> Monitor::monitorCollisionWithObjects(){

    std::vector<std::vector<double>> distances_to_objects;

    // For every obstacle calculate the distaces to each link
    for (int i = 0; i < this->obstacles.size(); i++ ) {
        
        std::vector<double> distances;

        for (int j = 0; j < this->arm->links.size(); j++) {
            distances.push_back(this->arm->links[j]->getShortestDistance( 
                this->obstacles[i] ));
        }
        
        distances_to_objects.push_back(distances);
    }

    #ifdef DEBUG
    // prints the distances calculated
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

    // For every link calculate the distance to other links
    for (int i = 0; i < this->arm->links.size(); i++) {

        std::vector<double> distances;
        for (int j = 0; j < this->arm->links.size(); j++) {
            if (i != j) {
                distances.push_back(this->arm->links[i]->getShortestDistance(
                    this->arm->links[j]));
            } else {
                distances.push_back(0);
            }
        }
        distances_to_objects.push_back(distances);
    }

    #ifdef DEBUG
    // print the calculated distances.
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