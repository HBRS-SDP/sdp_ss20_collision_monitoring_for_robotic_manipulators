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

    #ifdef DEBUG
        std::cout <<  "lambda: " << lambda << std::endl;
        std::cout <<  "vertex: " << vertex << std::endl;
    #endif //DEBUG
    
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

    distance = (vertex - m).norm();
    
    #ifdef DEBUG
        std::cout <<  "m: " << std::endl << m << std::endl;
        std::cout <<  "distance: " << distance << std::endl;
    #endif //DEBUG

    return distance;
}

double Line::getShortestDistanceToLine(Line line){
    Eigen::Vector3d basePointProjected, endPointProjected, midPoint;
    double shortestDistance;

    basePointProjected = this->projectionPoint( line.getBasePoint() );
    endPointProjected = this->projectionPoint( line.getEndPoint() );
    midPoint =  (this->endPoint + this->basePoint) / 2;

    #ifdef DEBUG
        std::cout <<  "basePoint: " << std::endl << line.getBasePoint() << std::endl;
        std::cout <<  "endPoint: " << std::endl << line.getEndPoint() << std::endl;
        std::cout <<  "basePointProjected: " << std::endl << basePointProjected << std::endl;
        std::cout <<  "endPointProjected: " << std::endl << endPointProjected << std::endl;
    #endif //DEBUG

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

float Cylinder::getLength(){
    return this->length;
}

float Cylinder::getRadius(){
    return this->radius;
}


double Cylinder::getShortestDistance(Primitive *primitive){
    Cylinder *cylinder = dynamic_cast<Cylinder*>(primitive);
    if(cylinder){
        this->getShortestDistance(cylinder);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDistance(sphere);
        }else{

        }
    }
}

double Cylinder::getShortestDistance(Cylinder *cylinder){    
    double shortestDistance = 0;
    Eigen::Vector3d startObstacle, endObstacle, basePoint, endPoint;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCylinder(0, 0, this->length, 1);
    Eigen::Vector4d zDirectionObstacle(0, 0, cylinder->length, 1);

    basePoint = (this->pose * origin).head(3);
    endPoint  = (this->pose * zDirectionCylinder).head(3);

    Line axisOfSymmetryCylinder(basePoint, endPoint);

    
    startObstacle = (cylinder->pose * origin).head(3);
    endObstacle  = (cylinder->pose * zDirectionObstacle).head(3);

    Line axisOfSymmetryObstacle(startObstacle, endObstacle);
    
    shortestDistance = axisOfSymmetryCylinder.getShortestDistanceToLine(axisOfSymmetryObstacle) - this->radius - cylinder->radius;

    #ifdef DEBUG
        std::cout << "st_c: " << std::endl << basePoint << std::endl;
        std::cout << "ed_c: " << std::endl << endPoint << std::endl;#
        std::cout << "st_o: " << std::endl << startObstacle << std::endl;
        std::cout << "ed_o: " << std::endl << endObstacle << std::endl;
        std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;
    #endif //DEBUG

    return shortestDistance;
}

double Cylinder::getShortestDistance(Sphere *sphere){
    double shortestDistance = 0;
    Eigen::Vector3d basePoint, endPoint, sphereCenter;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCylinder(0, 0, this->length, 1);

    basePoint = (this->pose * origin).head(3);
    endPoint  = (this->pose * zDirectionCylinder).head(3);

    Line axisOfSymmetryCylinder(basePoint, endPoint);
    
    sphereCenter = (sphere->pose * origin).head(3);

    shortestDistance = axisOfSymmetryCylinder.getShortestDistanceToVertex(sphereCenter) - this->radius - sphere->getRadius();

    #ifdef DEBUG
        std::cout << "st_c: " << std::endl << basePoint << std::endl;
        std::cout << "ed_c: " << std::endl << endPoint << std::endl;
        std::cout << "st_o: " << std::endl << sphereCenter << std::endl;
        std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;
    #endif //DEBUG

    return shortestDistance;
}

Sphere::Sphere(Eigen::Matrix4d pose, double radius){
    this->pose = pose;
    this->radius = radius;
}

Sphere::~Sphere(){

}

float Sphere::getRadius(){
    return this->radius;
}

double Sphere::getShortestDistance(Primitive *primitive){
    Cylinder *cylinder = dynamic_cast<Cylinder*>(primitive);
    if(cylinder){
        this->getShortestDistance(cylinder);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDistance(sphere);
        }else{

        }
    }
}

double Sphere::getShortestDistance(Cylinder *cylinder){
    double shortestDistance = 0;
    Eigen::Vector3d basePoint, endPoint, sphereCenter;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCylinder(0, 0, cylinder->getLength(), 1);

    basePoint = (cylinder->pose * origin).head(3);
    endPoint  = (cylinder->pose * zDirectionCylinder).head(3);

    Line axisOfSymmetryCylinder(basePoint, endPoint);
    
    sphereCenter = (this->pose * origin).head(3);

    shortestDistance = axisOfSymmetryCylinder.getShortestDistanceToVertex(sphereCenter) - cylinder->getRadius() - this->getRadius();

    #ifdef DEBUG
        std::cout << "st_c: " << std::endl << basePoint << std::endl;
        std::cout << "ed_c: " << std::endl << endPoint << std::endl;
        std::cout << "st_o: " << std::endl << sphereCenter << std::endl;
        std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;
    #endif //DEBUG

    return shortestDistance;
}

double Sphere::getShortestDistance(Sphere *sphere){
    double shortestDistance;
    Eigen::Vector3d obstacleSphereCenter, myCenter;
    Eigen::Vector4d origin(0, 0, 0, 1);

    obstacleSphereCenter = (sphere->pose * origin).head(3);
    myCenter = (this->pose * origin).head(3);

    shortestDistance = (obstacleSphereCenter - myCenter).norm() - sphere->getRadius() - this->radius;

    #ifdef DEBUG
        std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;
    #endif //DEBUG

    return shortestDistance;
}
