#include "primitives.h"
#include <math.h> 
#include <iostream>


Edge::Edge(Eigen::Vector3d basePoint, Eigen::Vector3d endPoint){
    this->basePoint = basePoint;
    this->endPoint = endPoint;
}

Edge::~Edge(){


}

Eigen::Vector3d Edge::getBasePoint(){
    return this->basePoint;
}

Eigen::Vector3d Edge::getEndPoint(){
    return this->endPoint;
}

Eigen::Vector3d Edge::projectionPoint(Eigen::Vector3d point){
    Eigen::Vector3d projectedPoint, normal, midPoint;

    normal = this->endPoint - this->basePoint;
    midPoint = (this->endPoint + this->basePoint) / 2;
    normal /= normal.norm();

    projectedPoint = point - (point - midPoint).dot(normal) * (normal);

    return projectedPoint;
}

double Edge::getShortestDistanceVertex(Eigen::Vector3d vertex){
    Eigen::Vector3d m;
    double lambda, x, y, z, p1, p2, length;
    double distance = 0;

    length = (this->endPoint - this->basePoint).norm();

    lambda = (vertex - this->basePoint).dot(this->endPoint - this->basePoint) / pow(length, 2);
    std::cout <<  "lambda: " << lambda << std::endl;
    std::cout <<  "vertex: " << vertex << std::endl;
    
    if(lambda <= 0){
        m = this->basePoint;
    }else if(lambda >= 1){
        m = this->endPoint;
    }else{
        x = this->basePoint[0] + lambda * (this->endPoint[0] - this->basePoint[0]);
        y = this->basePoint[1] + lambda * (this->endPoint[1] - this->basePoint[1]);
        z = this->basePoint[2] + lambda * (this->endPoint[2] - this->basePoint[2]);

        m = Eigen::Vector3d(x, y, z);
    }

    std::cout <<  "m: " << std::endl << m << std::endl;

    distance = (vertex - m).norm();
    std::cout <<  "distance: " << distance << std::endl;

    return distance;
}

double Edge::getShortestDistanceEdge(Edge edge){
    Eigen::Vector3d basePointProjected, endPointProjected, midPoint;
    double shortestDistance;

    basePointProjected = this->projectionPoint( edge.getBasePoint() );
    endPointProjected = this->projectionPoint( edge.getEndPoint() );
    midPoint =  (this->endPoint + this->basePoint) / 2;

    std::cout <<  "basePoint: " << std::endl << edge.getBasePoint() << std::endl;
    std::cout <<  "endPoint: " << std::endl << edge.getEndPoint() << std::endl;

    std::cout <<  "basePointProjected: " << std::endl << basePointProjected << std::endl;
    std::cout <<  "endPointProjected: " << std::endl << endPointProjected << std::endl;

    Edge projectedEdge(basePointProjected, endPointProjected);
    
    shortestDistance = projectedEdge.getShortestDistanceVertex(midPoint);

    return shortestDistance;
}

Cylinder::Cylinder(Eigen::Matrix4d pose, double length, double radius){
    this->pose = pose;
    this->length = length;
    this->radius = radius;
}

Cylinder::~Cylinder(){

}


double Cylinder::getShortestDistance(Primitive *obstacle){
    Cylinder *obstacleCylinder = dynamic_cast<Cylinder*>(obstacle);
    double shortestDistance = 0;
    Eigen::Vector3d startObstacle, endObstacle, basePoint, endPoint;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCylinder(0, 0, this->length, 1);
    Eigen::Vector4d zDirectionObstacle(0, 0, obstacleCylinder->length, 1);

    basePoint = (this->pose * origin).head(3);
    endPoint  = (this->pose * zDirectionCylinder).head(3);

    Edge axisOfSymmetryCylinder(basePoint, endPoint);

    std::cout << "st_c: " << std::endl << basePoint << std::endl;
    std::cout << "ed_c: " << std::endl << endPoint << std::endl;
    
    startObstacle = (obstacleCylinder->pose * origin).head(3);
    endObstacle  = (obstacleCylinder->pose * zDirectionObstacle).head(3);

    Edge axisOfSymmetryObstacle(startObstacle, endObstacle);

    std::cout << "st_o: " << std::endl << startObstacle << std::endl;
    std::cout << "ed_o: " << std::endl << endObstacle << std::endl;

    
    shortestDistance = axisOfSymmetryCylinder.getShortestDistanceEdge(axisOfSymmetryObstacle) - this->radius - obstacleCylinder->radius;

    std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;

    return shortestDistance;
}

// std::vector<double> Cylinder::getClosestPoint(std::vector<double>){

//     std::vector<double> return_var = {0., 0.};
//     return return_var;
// }


// N ellipsoide funcitons and declaration

// N_ellipsoid::N_ellipsoid(std::vector<double> pose){

// }

// N_ellipsoid::~N_ellipsoid(){

// }

// double N_ellipsoid::getShortestDistance(N_ellipsoid *){

//     return 0;
// }

// std::vector<double> N_ellipsoid::getClosestPoint(std::vector<double>){

// }
