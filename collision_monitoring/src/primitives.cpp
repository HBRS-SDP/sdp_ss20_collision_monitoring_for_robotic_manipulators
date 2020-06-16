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

Eigen::Vector3d Line::getClosestPointToPoint(Eigen::Vector3d point){
    Eigen::Vector3d closestPoint;
    double lambda, x, y, z, p1, p2, length;

    length = (this->endPoint - this->basePoint).norm();

    lambda = (point - this->basePoint).dot(this->endPoint - this->basePoint) / pow(length, 2);

    #ifdef DEBUG
        std::cout <<  "lambda: " << lambda << std::endl;
        std::cout <<  "point: " << point << std::endl;
    #endif //DEBUG
    
    if(lambda <= 0){
        closestPoint = this->basePoint;
    }else if(lambda >= 1){
        closestPoint = this->endPoint;
    }else{
        x = this->basePoint[0] + lambda * (this->endPoint[0] - this->basePoint[0]);
        y = this->basePoint[1] + lambda * (this->endPoint[1] - this->basePoint[1]);
        z = this->basePoint[2] + lambda * (this->endPoint[2] - this->basePoint[2]);

        closestPoint = Eigen::Vector3d(x, y, z);
    }


    return closestPoint;
}

double Line::getShortestDistanceToPoint(Eigen::Vector3d point){
    Eigen::Vector3d closestPoint;
    double distance = 0;
    
    this->getClosestPointToPoint(point);

    distance = (point - closestPoint).norm();
    
    #ifdef DEBUG
        std::cout <<  "closestPoint: " << std::endl << closestPoint << std::endl;
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
    
    shortestDistance = projectedLine.getShortestDistanceToPoint(midPoint);

    return shortestDistance;
}




Capsule::Capsule(Eigen::Matrix4d pose, double length, double radius){
    this->pose = pose;
    this->length = length;
    this->radius = radius;
}

Capsule::~Capsule(){

}

float Capsule::getLength(){
    return this->length;
}

float Capsule::getRadius(){
    return this->radius;
}


double Capsule::getShortestDistance(Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getShortestDistance(capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDistance(sphere);
        }else{

        }
    }
}

double Capsule::getShortestDistance(Capsule *capsule){    
    double shortestDistance = 0;
    double lambdaM1, lambdaM2;
    Eigen::Vector3d basePointObstacle, endPointObstacle, basePointOwn, endPointOwn;
    Capsule *capsuleOwn;
    Capsule *capsuleObstacle;

    if(this->length > capsule->getLength()){
        capsuleOwn = this;
        capsuleObstacle = capsule;
    }
    else{
        capsuleOwn = capsule;
        capsuleObstacle = this;
    }
    
    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionOwn(0, 0, capsuleOwn->length, 1);
    Eigen::Vector4d zDirectionObstacle(0, 0, capsuleObstacle->length, 1);

    basePointOwn = (capsuleOwn->pose * origin).head(3);
    endPointOwn  = (capsuleOwn->pose * zDirectionOwn).head(3);

    Line axisOfSymmetryOwn(basePointOwn, endPointOwn);

    basePointObstacle = (capsuleObstacle->pose * origin).head(3);
    endPointObstacle  = (capsuleObstacle->pose * zDirectionObstacle).head(3);

    Line axisOfSymmetryObstacle(basePointObstacle, endPointObstacle);


    lambdaM1 = (basePointObstacle - basePointOwn).dot(endPointOwn - basePointOwn) / pow(capsuleOwn->getLength(), 2);
    lambdaM2 = (endPointObstacle - basePointOwn).dot(endPointOwn - basePointOwn) / pow(capsuleOwn->getLength(), 2);

    if(lambdaM1 >= 0 && lambdaM1 <= 1){
        if(lambdaM2 >=0 && lambdaM2 <= 1){
            //m1 and m2 inside
            shortestDistance = axisOfSymmetryOwn.getShortestDistanceToLine(axisOfSymmetryObstacle);
        }else{
            // //m1 inside
            #ifdef DEBUG
                std::cout << "m1 inside" << std::endl;
                std::cout << basePointObstacle << std::endl;
                std::cout << axisOfSymmetryOwn.getShortestDistanceToPoint(basePointObstacle) << std::endl;
                std::cout << axisOfSymmetryObstacle.getShortestDistanceToPoint(endPointOwn) << std::endl;
            #endif //DEBUG
            if( axisOfSymmetryOwn.getShortestDistanceToPoint(basePointObstacle) < axisOfSymmetryObstacle.getShortestDistanceToPoint(endPointOwn) ){
                shortestDistance = axisOfSymmetryOwn.getShortestDistanceToLine(axisOfSymmetryObstacle);
            }else{
                shortestDistance = axisOfSymmetryObstacle.getShortestDistanceToLine(axisOfSymmetryOwn);
            }
        }
    }else if(lambdaM2 >=0 && lambdaM2 <= 1){
        //m2 inside
        #ifdef DEBUG
            std::cout << "m2 inside" << std::endl;
            std::cout << endPointObstacle << std::endl;
            std::cout << axisOfSymmetryOwn.getShortestDistanceToPoint(endPointObstacle) << std::endl;
            std::cout << axisOfSymmetryObstacle.getShortestDistanceToPoint(basePointOwn) << std::endl;
        #endif //DEBUG
        if( axisOfSymmetryOwn.getShortestDistanceToPoint(endPointObstacle) < axisOfSymmetryObstacle.getShortestDistanceToPoint(basePointOwn) ){
            shortestDistance = axisOfSymmetryOwn.getShortestDistanceToLine(axisOfSymmetryObstacle);
        }else{
            shortestDistance = axisOfSymmetryObstacle.getShortestDistanceToLine(axisOfSymmetryOwn);
        }
    }else{
        //m1 and m2 outside
        shortestDistance = axisOfSymmetryObstacle.getShortestDistanceToLine(axisOfSymmetryOwn);
    }

    shortestDistance = shortestDistance - capsuleOwn->getRadius() - capsuleObstacle->getRadius();

    #ifdef DEBUG
        std::cout << "st_c: " << std::endl << basePoint << std::endl;
        std::cout << "ed_c: " << std::endl << endPoint << std::endl;#
        std::cout << "st_o: " << std::endl << startObstacle << std::endl;
        std::cout << "ed_o: " << std::endl << endObstacle << std::endl;
        std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;
    #endif //DEBUG

    return shortestDistance;
}

double Capsule::getShortestDistance(Sphere *sphere){
    double shortestDistance = 0;
    Eigen::Vector3d basePoint, endPoint, sphereCenter;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCapsule(0, 0, this->length, 1);

    basePoint = (this->pose * origin).head(3);
    endPoint  = (this->pose * zDirectionCapsule).head(3);

    Line axisOfSymmetryCapsule(basePoint, endPoint);
    
    sphereCenter = (sphere->pose * origin).head(3);

    shortestDistance = axisOfSymmetryCapsule.getShortestDistanceToPoint(sphereCenter) - this->radius - sphere->getRadius();

    #ifdef DEBUG
        std::cout << "st_c: " << std::endl << basePoint << std::endl;
        std::cout << "ed_c: " << std::endl << endPoint << std::endl;
        std::cout << "st_o: " << std::endl << sphereCenter << std::endl;
        std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;
    #endif //DEBUG

    return shortestDistance;
}

Eigen::Vector3d Capsule::getShortestDirection(Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getShortestDirection(capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDirection(sphere);
        }else{

        }
    }
}

Eigen::Vector3d Capsule::getShortestDirection(Capsule *capsule){
    Eigen::Vector3d shortestDirection;
    return shortestDirection;
}

Eigen::Vector3d Capsule::getShortestDirection(Sphere *sphere){
    Eigen::Vector3d shortestDirection;
    return shortestDirection;
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
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getShortestDistance(capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDistance(sphere);
        }else{

        }
    }
}

double Sphere::getShortestDistance(Capsule *capsule){
    double shortestDistance = 0;
    Eigen::Vector3d basePoint, endPoint, sphereCenter;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCapsule(0, 0, capsule->getLength(), 1);

    basePoint = (capsule->pose * origin).head(3);
    endPoint  = (capsule->pose * zDirectionCapsule).head(3);

    Line axisOfSymmetryCapsule(basePoint, endPoint);
    
    sphereCenter = (this->pose * origin).head(3);

    shortestDistance = axisOfSymmetryCapsule.getShortestDistanceToPoint(sphereCenter) - capsule->getRadius() - this->getRadius();

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
    Eigen::Vector3d obstacleSphereCenter, ownCenter;
    Eigen::Vector4d origin(0, 0, 0, 1);

    obstacleSphereCenter = (sphere->pose * origin).head(3);
    ownCenter = (this->pose * origin).head(3);

    shortestDistance = (obstacleSphereCenter - ownCenter).norm() - sphere->getRadius() - this->radius;

    #ifdef DEBUG
        std::cout << "shortestDistance: " << std::endl << shortestDistance << std::endl;
    #endif //DEBUG

    return shortestDistance;
}

Eigen::Vector3d Sphere::getShortestDirection(Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getShortestDirection(capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDirection(sphere);
        }else{

        }
    }
}

Eigen::Vector3d Sphere::getShortestDirection(Capsule *capsule){
    Eigen::Vector3d shortestDirection;

    double shortestDistance = 0;
    Eigen::Vector3d basePoint, endPoint, sphereCenter;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCapsule(0, 0, capsule->getLength(), 1);

    basePoint = (capsule->pose * origin).head(3);
    endPoint  = (capsule->pose * zDirectionCapsule).head(3);

    Line axisOfSymmetryCapsule(basePoint, endPoint);
    
    sphereCenter = (this->pose * origin).head(3);

    shortestDistance = axisOfSymmetryCapsule.getShortestDistanceToPoint(sphereCenter) - capsule->getRadius() - this->getRadius();


    return shortestDirection;
}

Eigen::Vector3d Sphere::getShortestDirection(Sphere *sphere){
    Eigen::Vector3d shortestDirection;
    Eigen::Vector3d obstacleSphereCenter, ownCenter;
    Eigen::Vector4d origin(0, 0, 0, 1);

    obstacleSphereCenter = (sphere->pose * origin).head(3);
    ownCenter = (this->pose * origin).head(3);

    shortestDirection = obstacleSphereCenter - ownCenter;    

    return shortestDirection;
}