#include "monitor.h"
#include <vector>

Monitor::Monitor(Arm* arm){
    this->arm = arm;
}

Monitor::~Monitor(){
    std::cout << "###" << this->obstacles.size() << std::endl;
    for( int i=0; i < this->obstacles.size(); i++){
        std::cout << "###" << this->obstacles[i] << std::endl;
        std::cout << "###" << this->obstacles[i]->pose << std::endl;
        delete obstacles[i];
        obstacles[i] = NULL;
    }
}

void Monitor::addObstacle(Primitive* obstacle) {
    Capsule *capsule = dynamic_cast<Capsule*>(obstacle);
    if(capsule){
        this->addObstacle(capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(obstacle);
        if(sphere){
            this->addObstacle(sphere);
        }
    }

}

void Monitor::addObstacle(Sphere* obstacle) {
    Sphere* obstacleCopy = new Sphere(obstacle);
    obstacles.push_back(obstacleCopy);
}

void Monitor::addObstacle(Capsule* obstacle) {
    Capsule* obstacleCopy = new Capsule(obstacle);
    obstacles.push_back(obstacleCopy);
}


void Monitor::addObstacle(Arm* arm) {
    // Adds every link of the arm (a primitive) to the obstacles vector.
    for (int i = 0; i < arm->links.size(); i++) {
        // this->obstacles.push_back(arm->links[i]);
        this->addObstacle(arm->links[i]);
    }
}

std::vector<std::vector<double>> Monitor::distanceToObjects(){

    std::vector<std::vector<double>> distanceToObjects;

    // For every obstacle calculate the distaces to each link
    for (int i = 0; i < this->obstacles.size(); i++ ) {
        
        std::vector<double> distances;

        for (int j = 0; j < this->arm->links.size(); j++) {
            distances.push_back(this->arm->links[j]->getShortestDistance( 
                this->obstacles[i] ));
        }
        
        distanceToObjects.push_back(distances);
    }

    #ifdef DEBUG
    // prints the distances calculated
    for (int i = 0; i < distanceToObjects.size(); i++) {
        std::cout << "distances to obstacle [" << i << "]:";

        for (int j = 0; j < distanceToObjects[i].size(); j++){
            std::cout << distanceToObjects[i][j] << " "; 
        }
        std::cout << std::endl;
    }
    #endif //DEBUG

    return distanceToObjects;
}

std::vector<std::vector<double>> Monitor::distanceBetweenArmLinks(){
    
    std::vector<std::vector<double>> distanceToObjects;

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
        distanceToObjects.push_back(distances);
    }

    #ifdef DEBUG
    // print the calculated distances.
    for (int i = 0; i < distanceToObjects.size(); i++) {
        std::cout << "distances from link [" << i << "] to other links:";

        for (int j = 0; j < distanceToObjects[i].size(); j++){
            std::cout << distanceToObjects[i][j] << " "; 
        }
        std::cout << std::endl;
    }
    #endif //DEBUG

    return distanceToObjects;
}