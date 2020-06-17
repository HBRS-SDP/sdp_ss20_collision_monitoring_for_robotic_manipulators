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

void Line::getClosestPointsBetweenLines(Eigen::MatrixXd &closestPoints, Line line){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::Vector3d obstacleProjectedClosestPoint;

    Eigen::Vector3d basePointProjected, endPointProjected, midPoint;
    double shortestDistance;
    double ratio;

    basePointProjected = this->projectionPoint( line.getBasePoint() );
    endPointProjected = this->projectionPoint( line.getEndPoint() );
    midPoint =  (this->endPoint + this->basePoint) / 2;

    Line projectedLine(basePointProjected, endPointProjected);
    
    obstacleProjectedClosestPoint = projectedLine.getClosestPointToPoint(midPoint);

    ratio = (obstacleProjectedClosestPoint - basePointProjected).norm() / (endPointProjected - basePointProjected).norm();
    obstacleClosestPoint = (line.getEndPoint() - line.getBasePoint()) * ratio + line.getBasePoint();
    ownClosestPoint = this->getClosestPointToPoint(obstacleClosestPoint);

    closestPoints.row(0) = ownClosestPoint;
    closestPoints.row(1) = obstacleClosestPoint;
}

double Line::getShortestDistanceToPoint(Eigen::Vector3d point){
    Eigen::Vector3d closestPoint;
    double distance = 0;
    
    closestPoint = this->getClosestPointToPoint(point);

    distance = (point - closestPoint).norm();

    return distance;
}

double Line::getShortestDistanceToLine(Line line){
    Eigen::Vector3d basePointProjected, endPointProjected, midPoint;
    double shortestDistance;

    basePointProjected = this->projectionPoint( line.getBasePoint() );
    endPointProjected = this->projectionPoint( line.getEndPoint() );
    midPoint =  (this->endPoint + this->basePoint) / 2;

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
        return this->getShortestDistance(capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            return this->getShortestDistance(sphere);
        }else{

        }
    }
}

double Capsule::getShortestDistance(Capsule *capsule){    
    double shortestDistance = 0;
    Eigen::Vector3d shortestDirection;

    this->getShortestDirection(shortestDirection, capsule);
    shortestDistance = shortestDirection.norm() - this->radius - capsule->getRadius();

    return shortestDistance;
}

double Capsule::getShortestDistance(Sphere *sphere){
    double shortestDistance = 0;
    Eigen::Vector3d shortestDirection;

    this->getShortestDirection(shortestDirection, sphere);
    shortestDistance = shortestDirection.norm() - this->radius - sphere->getRadius();

    return shortestDistance;
}

void Capsule::getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getShortestDirection(shortestDirection, capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDirection(shortestDirection, sphere);
        }else{

        }
    }
}

void Capsule::getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    
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
            axisOfSymmetryOwn.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryObstacle);
            ownClosestPoint = closestPoints.row(0);
            obstacleClosestPoint = closestPoints.row(1);
        }else{
            //m1 inside
            if( axisOfSymmetryOwn.getShortestDistanceToPoint(basePointObstacle) < axisOfSymmetryObstacle.getShortestDistanceToPoint(endPointOwn) ){
                axisOfSymmetryOwn.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryObstacle);
                ownClosestPoint = closestPoints.row(0);
                obstacleClosestPoint = closestPoints.row(1);
            }else{
                axisOfSymmetryObstacle.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryOwn);
                obstacleClosestPoint = closestPoints.row(0);
                ownClosestPoint = closestPoints.row(1);
            }
        }   
    }else if(lambdaM2 >=0 && lambdaM2 <= 1){
        //m2 inside
        if( axisOfSymmetryOwn.getShortestDistanceToPoint(endPointObstacle) < axisOfSymmetryObstacle.getShortestDistanceToPoint(basePointOwn) ){
            axisOfSymmetryOwn.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryObstacle);
            ownClosestPoint = closestPoints.row(0);
            obstacleClosestPoint = closestPoints.row(1);
        }else{
            axisOfSymmetryObstacle.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryOwn);
            obstacleClosestPoint = closestPoints.row(0);
            ownClosestPoint = closestPoints.row(1);
        }
    }else{
        //m1 and m2 outside
        axisOfSymmetryObstacle.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryOwn);
        obstacleClosestPoint = closestPoints.row(0);
        ownClosestPoint = closestPoints.row(1);
    }
    
    shortestDirection = obstacleClosestPoint - ownClosestPoint;
}

void Capsule::getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere){
    Eigen::Vector3d basePoint, endPoint, sphereCenter, closestPoint;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCapsule(0, 0, this->length, 1);

    basePoint = (this->pose * origin).head(3);
    endPoint  = (this->pose * zDirectionCapsule).head(3);

    Line axisOfSymmetryCapsule(basePoint, endPoint);
    
    sphereCenter = (sphere->pose * origin).head(3);
    closestPoint = axisOfSymmetryCapsule.getClosestPointToPoint(sphereCenter);
    shortestDirection = closestPoint - sphereCenter;
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
        return this->getShortestDistance(capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            return this->getShortestDistance(sphere);
        }else{

        }
    }
}

double Sphere::getShortestDistance(Capsule *capsule){
    double shortestDistance = 0;
    Eigen::Vector3d shortestDirection;
    this->getShortestDirection(shortestDirection, capsule);

    shortestDistance = shortestDirection.norm() - capsule->getRadius() - this->getRadius();

    return shortestDistance;
}

double Sphere::getShortestDistance(Sphere *sphere){
    double shortestDistance;
    Eigen::Vector3d shortestDirection;
    
    this->getShortestDirection(shortestDirection ,sphere);
    shortestDistance = shortestDirection.norm() - sphere->getRadius() - this->radius;

    return shortestDistance;
}

void Sphere::getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getShortestDirection(shortestDirection, capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDirection(shortestDirection, sphere);
        }else{

        }
    }
}

void Sphere::getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule){
    Eigen::Vector3d basePoint, endPoint, sphereCenter, closestPoint;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCapsule(0, 0, capsule->getLength(), 1);

    basePoint = (capsule->pose * origin).head(3);
    endPoint  = (capsule->pose * zDirectionCapsule).head(3);

    Line axisOfSymmetryCapsule(basePoint, endPoint);
    
    sphereCenter = (this->pose * origin).head(3);
    closestPoint = axisOfSymmetryCapsule.getClosestPointToPoint(sphereCenter);
    shortestDirection = closestPoint - sphereCenter;
}

void Sphere::getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere){
    Eigen::Vector3d obstacleSphereCenter, ownCenter;
    Eigen::Vector4d origin(0, 0, 0, 1);

    obstacleSphereCenter = (sphere->pose * origin).head(3);
    ownCenter = (this->pose * origin).head(3);

    shortestDirection = obstacleSphereCenter - ownCenter;
}