#include "primitives.h"
#include <math.h> 
#include <iostream>


Line::Line(Eigen::Vector3d basePoint, Eigen::Vector3d endPoint){
    this->basePoint = basePoint;
    this->endPoint = endPoint;
}

Line::~Line(){


}

Eigen::Vector3d Line::getBasePoint(){
    return this->basePoint;
}

Eigen::Vector3d Line::getEndPoint(){
    return this->endPoint;
}

Eigen::Vector3d Line::projectionPoint(Eigen::Vector3d point){
    Eigen::Vector3d projectedPoint, normal, midPoint;

    normal = this->endPoint - this->basePoint;
    midPoint = (this->endPoint + this->basePoint) / 2;
    normal /= normal.norm();

    projectedPoint = point - (point - midPoint).dot(normal) * (normal);

    return projectedPoint;
}

double Line::getShortestDistanceToVertex(Eigen::Vector3d vertex){
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

double Line::getShortestDistanceToLine(Line line){
    Eigen::Vector3d basePointProjected, endPointProjected, midPoint;
    double shortestDistance;

    basePointProjected = this->projectionPoint( line.getBasePoint() );
    endPointProjected = this->projectionPoint( line.getEndPoint() );
    midPoint =  (this->endPoint + this->basePoint) / 2;

    std::cout <<  "basePoint: " << std::endl << line.getBasePoint() << std::endl;
    std::cout <<  "endPoint: " << std::endl << line.getEndPoint() << std::endl;

    std::cout <<  "basePointProjected: " << std::endl << basePointProjected << std::endl;
    std::cout <<  "endPointProjected: " << std::endl << endPointProjected << std::endl;

    Line projectedLine(basePointProjected, endPointProjected);
    
    shortestDistance = projectedLine.getShortestDistanceToVertex(midPoint);

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

    Line axisOfSymmetryCylinder(basePoint, endPoint);

    std::cout << "st_c: " << std::endl << basePoint << std::endl;
    std::cout << "ed_c: " << std::endl << endPoint << std::endl;
    
    startObstacle = (obstacleCylinder->pose * origin).head(3);
    endObstacle  = (obstacleCylinder->pose * zDirectionObstacle).head(3);

    Line axisOfSymmetryObstacle(startObstacle, endObstacle);

    std::cout << "st_o: " << std::endl << startObstacle << std::endl;
    std::cout << "ed_o: " << std::endl << endObstacle << std::endl;

    
    shortestDistance = axisOfSymmetryCylinder.getShortestDistanceToLine(axisOfSymmetryObstacle) - this->radius - obstacleCylinder->radius;

    std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;

    return shortestDistance;
}

// std::vector<double> Cylinder::getClosestPoint(std::vector<double>){

//     std::vector<double> return_var = {0., 0.};
//     return return_var;
// }


// N ellipsoide functions and declaration

// N_ellipsoid::N_ellipsoid(std::vector<double> pose){

// }

// N_ellipsoid::~N_ellipsoid(){

// }

// double N_ellipsoid::getShortestDistance(N_ellipsoid *){

//     return 0;
// }

// std::vector<double> N_ellipsoid::getClosestPoint(std::vector<double>){

// }
