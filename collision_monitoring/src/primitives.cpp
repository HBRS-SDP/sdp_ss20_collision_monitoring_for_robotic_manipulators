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

    if( (basePointProjected - endPointProjected).norm() == 0 ){
        //Parallel lines    
        obstacleProjectedClosestPoint = basePointProjected;

        ownClosestPoint = midPoint;
        obstacleClosestPoint = (line.endPoint + line.basePoint) / 2;
    }else{
        obstacleProjectedClosestPoint = projectedLine.getClosestPointToPoint(midPoint);

        ratio = (obstacleProjectedClosestPoint - basePointProjected).norm() / (endPointProjected - basePointProjected).norm();
        obstacleClosestPoint = (line.getEndPoint() - line.getBasePoint()) * ratio +  line.getBasePoint();
        ownClosestPoint = this->getClosestPointToPoint(obstacleClosestPoint);
    }
    
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

Capsule::Capsule(Capsule* capsule){
    this->pose = capsule->pose;
    this->length = capsule->getLength();
    this->radius = capsule->getRadius();
}

Capsule::~Capsule(){

}

float Capsule::getLength(){
    return this->length;
}

float Capsule::getRadius(){
    return this->radius;
}

void Capsule::getClosestPoints(Eigen::MatrixXd &closestPoints, Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getClosestPoints(closestPoints, capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getClosestPoints(closestPoints, sphere);
        }else{
            Box3 *box = dynamic_cast<Box3*>(primitive);
        if(box){
            this->getClosestPoints(closestPoints, box);
        }

        }
    }
}

void Capsule::getClosestPoints(Eigen::MatrixXd &closestPoints, Capsule *capsule){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    
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
            closestPoints.row(0).swap(closestPoints.row(1));
        }else{
            //m1 inside
            if( axisOfSymmetryOwn.getShortestDistanceToPoint(basePointObstacle) < axisOfSymmetryObstacle.getShortestDistanceToPoint(endPointOwn) ){
                axisOfSymmetryOwn.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryObstacle);
                closestPoints.row(0).swap(closestPoints.row(1));
            }else{
                axisOfSymmetryObstacle.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryOwn);
            }
        }   
    }else if(lambdaM2 >=0 && lambdaM2 <= 1){
        //m2 inside
        if( axisOfSymmetryOwn.getShortestDistanceToPoint(endPointObstacle) < axisOfSymmetryObstacle.getShortestDistanceToPoint(basePointOwn) ){
            axisOfSymmetryOwn.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryObstacle);
            closestPoints.row(0).swap(closestPoints.row(1));
        }else{
            axisOfSymmetryObstacle.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryOwn);
        }
    }else{
        //m1 and m2 outside
        if( axisOfSymmetryOwn.getShortestDistanceToPoint(endPointObstacle) < axisOfSymmetryOwn.getShortestDistanceToPoint(basePointObstacle) ){
            axisOfSymmetryObstacle.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryOwn);
        }else{
            axisOfSymmetryOwn.getClosestPointsBetweenLines(closestPoints, axisOfSymmetryObstacle);
            closestPoints.row(0).swap(closestPoints.row(1));
        }
    }
}

void Capsule::getClosestPoints(Eigen::MatrixXd &closestPoints, Sphere *sphere){
    Eigen::Vector3d basePoint, endPoint, ownClosestPoint, obstacleClosestPoint;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCapsule(0, 0, this->length, 1);

    basePoint = (this->pose * origin).head(3);
    endPoint  = (this->pose * zDirectionCapsule).head(3);

    Line axisOfSymmetryCapsule(basePoint, endPoint);
    
    obstacleClosestPoint = (sphere->pose * origin).head(3);
    ownClosestPoint = axisOfSymmetryCapsule.getClosestPointToPoint(obstacleClosestPoint);

    closestPoints.row(0) = ownClosestPoint;
    closestPoints.row(1) = obstacleClosestPoint;
}

void Capsule::getClosestPoints(Eigen::MatrixXd &closestPoints, Box3 *box){

    Eigen::Vector3d basePoint, endPoint, ownClosestPoint, obstacleClosestPoint;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCapsule(0, 0, this->length, 1);

    basePoint = (this->pose * origin).head(3);
    endPoint  = (this->pose * zDirectionCapsule).head(3);

    Line axisOfSymmetryCapsule(basePoint, endPoint);


    

    obstacleClosestPoint = box->OwnClosestPoint(&axisOfSymmetryCapsule);
    ownClosestPoint = axisOfSymmetryCapsule.getClosestPointToPoint(obstacleClosestPoint);

    closestPoints.row(0) = ownClosestPoint;
    closestPoints.row(1) = obstacleClosestPoint;
    
}
void Capsule::getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getShortestDirection(shortestDirection, capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDirection(shortestDirection, sphere);
        }else{Box3 *box = dynamic_cast<Box3*>(primitive);
        if(box){
            this->getShortestDirection(shortestDirection, box);
        }

        }
    }
}

void Capsule::getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    
    this->getClosestPoints(closestPoints, capsule);
    ownClosestPoint = closestPoints.row(0);
    obstacleClosestPoint = closestPoints.row(1);
    
    shortestDirection = obstacleClosestPoint - ownClosestPoint;
}
void Capsule::getShortestDirection(Eigen::Vector3d &shortestDirection, Box3 *box){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    this->getClosestPoints(closestPoints, box);
     ownClosestPoint = closestPoints.row(0);
    obstacleClosestPoint = closestPoints.row(1);
    shortestDirection = obstacleClosestPoint - ownClosestPoint;
}

void Capsule::getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    
    this->getClosestPoints(closestPoints, sphere);
    ownClosestPoint = closestPoints.row(0);
    obstacleClosestPoint = closestPoints.row(1);
    
    shortestDirection = obstacleClosestPoint - ownClosestPoint;
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

double Capsule::getShortestDistance(Box3 *box){
    double shortestDistance = 0;
    Eigen::Vector3d shortestDirection;
    this->getShortestDirection(shortestDirection, box);
    shortestDistance = shortestDirection.norm() - this->radius;
    return shortestDistance;
}
Sphere::Sphere(Eigen::Matrix4d pose, double radius){
    this->pose = pose;
    this->radius = radius;
}

Sphere::Sphere(Sphere* sphere) {
    this->pose = sphere->pose;
    this->radius = sphere->getRadius();
}

Sphere::~Sphere(){

}

float Sphere::getRadius(){
    return this->radius;
}

void Sphere::getClosestPoints(Eigen::MatrixXd &closestPoints, Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getClosestPoints(closestPoints, capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getClosestPoints(closestPoints, sphere);
        }else{

        }
    }
}

void Sphere::getClosestPoints(Eigen::MatrixXd &closestPoints, Capsule *capsule){
    Eigen::Vector3d basePoint, endPoint, ownClosestPoint, obstacleClosestPoint;

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d zDirectionCapsule(0, 0, capsule->getLength(), 1);

    basePoint = (capsule->pose * origin).head(3);
    endPoint  = (capsule->pose * zDirectionCapsule).head(3);

    Line axisOfSymmetryCapsule(basePoint, endPoint);
    
    ownClosestPoint = (this->pose * origin).head(3);
    obstacleClosestPoint = axisOfSymmetryCapsule.getClosestPointToPoint(ownClosestPoint);

    closestPoints.row(0) = ownClosestPoint;
    closestPoints.row(1) = obstacleClosestPoint;
}

void Sphere::getClosestPoints(Eigen::MatrixXd &closestPoints, Sphere *sphere){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::Vector4d origin(0, 0, 0, 1);

    obstacleClosestPoint = (sphere->pose * origin).head(3);
    ownClosestPoint = (this->pose * origin).head(3);

    closestPoints.row(0) = ownClosestPoint;
    closestPoints.row(1) = obstacleClosestPoint;
}

void Sphere::getClosestPoints(Eigen::MatrixXd &closestPoints, Box3 *box){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::Vector4d origin(0, 0, 0, 1);
    ownClosestPoint = (this->pose * origin).head(3);
    obstacleClosestPoint = box->ClosestPoint( ownClosestPoint);
    closestPoints.row(0) = ownClosestPoint;
    closestPoints.row(1) = obstacleClosestPoint;
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
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    
    this->getClosestPoints(closestPoints, capsule);
    ownClosestPoint = closestPoints.row(0);
    obstacleClosestPoint = closestPoints.row(1);
    
    shortestDirection = obstacleClosestPoint - ownClosestPoint;
}

void Sphere::getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    
    this->getClosestPoints(closestPoints, sphere);
    ownClosestPoint = closestPoints.row(0);
    obstacleClosestPoint = closestPoints.row(1);
    
    shortestDirection = obstacleClosestPoint - ownClosestPoint;
}

void Sphere::getShortestDirection(Eigen::Vector3d &shortestDirection, Box3 *box){
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    this->getClosestPoints(closestPoints, box);
    ownClosestPoint = closestPoints.row(0);
    obstacleClosestPoint = closestPoints.row(1);
    shortestDirection = obstacleClosestPoint - ownClosestPoint;
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
            Box3 *box = dynamic_cast<Box3*>(primitive);
        if(sphere){
            return this->getShortestDistance(box);
        }

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

double Sphere::getShortestDistance(Box3 *box){
    double shortestDistance;
    Eigen::Vector3d shortestDirection;
    this->getShortestDirection(shortestDirection ,box);
    shortestDistance = shortestDirection.norm() - this->getRadius() ;
    return shortestDistance;
}
// Adding my box -----------------------------------------------------------------------

  Box3::Box3(const Eigen::Vector3d &vmin, const Eigen::Vector3d &vmax) 
{
        bounds[0] = vmin;    // min point
        bounds[1] = vmax;    // max point
        minPoint = vmin;     // min point
        maxPoint = vmax;    // max point
        extents[0]=0.33;
        extents[1]=0.30;
        extents[2]=0.15;
        box_center= (minPoint + maxPoint) * 0.5f;
}

   Box3::Box3(Eigen::Vector3d &center)
    {
        box_center = center;
        minPoint = box_center -extents;
        maxPoint = box_center +extents;
    }
    Box3::Box3(Eigen::Matrix4d &pose, double x,double y,double z){
        extents[0]=x/2;
        extents[1]=y/2;
        extents[2]=z/2;
        Eigen::Vector4d origin(0, 0, 0, 1);
        box_center = (pose * origin).head(3);
        minPoint = box_center -extents;
        maxPoint = box_center +extents;

    }

    Box3::Box3(Eigen::Vector3d &pose, double x,double y,double z){
      extents[0]=x/2;
        extents[1]=y/2;
        extents[2]=z/2;
        Eigen::Vector4d origin(0, 0, 0, 1);
        box_center = pose;
        minPoint = box_center -extents;
        maxPoint = box_center +extents;
    }
    Box3::Box3(Box3* box){

   this->minPoint= box->minPoint;
   this->maxPoint= box->maxPoint;
   this->box_center= box->box_center;
    }
   Box3::~Box3(){}
    bool Box3::intersection ( const Ray &r) const 
            {
                float tmin, tmax, tymin, tymax, tzmin, tzmax; 
                tmin = (bounds[r.sign[0]][0] - r.orig[0]) * r.invdir[0]; 
                tmax = (bounds[1-r.sign[0]][0] - r.orig[0]) * r.invdir[0]; 
                tymin = (bounds[r.sign[1]][1] - r.orig[1]) * r.invdir[1]; 
                tymax = (bounds[1-r.sign[1]][1] - r.orig[1]) * r.invdir[1]; 
                if ((tmin > tymax) || (tymin > tmax)) 
                    return false; 
                if (tymin > tmin) 
                    tmin = tymin; 
                if (tymax < tmax) 
                    tmax = tymax; 
                tzmin = (bounds[r.sign[2]][2] - r.orig[2]) * r.invdir[2]; 
                tzmax = (bounds[1-r.sign[2]][2] - r.orig[2]) * r.invdir[2]; 
                if ((tmin > tzmax) || (tzmin > tmax)) 
                    return false; 
                if (tzmin > tmin) 
                    tmin = tzmin; 
                if (tzmax < tmax) 
                    tmax = tzmax; 
                return true;
            }   
  Line* Box3::Edge(int edgeIndex) const
    {        
         Eigen::Vector3d minPoint =bounds[0];
         Eigen::Vector3d maxPoint=bounds[1];
	    Line* result;
            switch(edgeIndex)
            {   case 0: result=new Line(minPoint, Eigen::Vector3d(minPoint[0], minPoint[1], maxPoint[2]));
                case 1: result=new Line(minPoint, Eigen::Vector3d(minPoint[0], maxPoint[1], minPoint[2]));
                case 2: result=new Line(minPoint, Eigen::Vector3d(maxPoint[0], minPoint[1], minPoint[2]));
                case 3: result=new Line(Eigen::Vector3d(minPoint[0], minPoint[1], maxPoint[2]), Eigen::Vector3d(minPoint[0], maxPoint[1], maxPoint[2]));
                case 4: result=new Line(Eigen::Vector3d(minPoint[0], minPoint[1], maxPoint[2]), Eigen::Vector3d(maxPoint[0], minPoint[1], maxPoint[2]));
                case 5: result=new Line(Eigen::Vector3d(minPoint[0], maxPoint[1], minPoint[2]), Eigen::Vector3d(minPoint[0], maxPoint[1], maxPoint[2]));
                case 6: result=new Line(Eigen::Vector3d(minPoint[0], maxPoint[1], minPoint[2]), Eigen::Vector3d(maxPoint[0], maxPoint[1], minPoint[2]));
                case 7: result=new Line(Eigen::Vector3d(minPoint[0], maxPoint[1], maxPoint[2]), maxPoint);
                case 8: result=new Line(Eigen::Vector3d(maxPoint[0], minPoint[1], minPoint[2]), Eigen::Vector3d(maxPoint[0], minPoint[1], maxPoint[2]));
                case 9: result=new Line(Eigen::Vector3d(maxPoint[0], minPoint[1], minPoint[2]), Eigen::Vector3d(maxPoint[0], maxPoint[1], minPoint[2]));
                case 10: result=new Line(Eigen::Vector3d(maxPoint[0], minPoint[1], maxPoint[2]), maxPoint);
                case 11: result=new Line(Eigen::Vector3d(maxPoint[0], maxPoint[1], minPoint[2]), maxPoint);
            } return result;
   }      
  std::vector<Line*>  Box3::EdgeList() const
    {
        std::vector<Line*> eArray;
                for(int i = 0; i < 12; ++i)
            {
                eArray.push_back(Edge(i));
            }
            return eArray;
    }
Eigen::Vector3d Box3:: CornerPoint(int cornerIndex) const
{
	switch(cornerIndex) 
    { 
		case 0: return minPoint;
		case 1: return Eigen::Vector3d(minPoint[0], minPoint[1], maxPoint[2]);
		case 2: return Eigen::Vector3d(minPoint[0], maxPoint[1], minPoint[2]);
		case 3: return Eigen::Vector3d(minPoint[0], maxPoint[1], maxPoint[2]);
		case 4: return Eigen::Vector3d(maxPoint[0], minPoint[1], minPoint[2]);
		case 5: return Eigen::Vector3d(maxPoint[0], minPoint[1], maxPoint[2]);
		case 6: return Eigen::Vector3d(maxPoint[0], maxPoint[1], minPoint[2]);
		case 7: return maxPoint;
	}
}
Eigen::Vector3d Box3:: SideCenterPoint(int sideIndex) const
{
	switch(sideIndex)
	{
	case 0: return Eigen::Vector3d(minPoint[0], box_center[1], box_center[2]);
	case 1: return Eigen::Vector3d(maxPoint[0], box_center[1], box_center[2]);
	case 2: return Eigen::Vector3d(box_center[0], minPoint[1], box_center[2]);
	case 3: return Eigen::Vector3d(box_center[0], maxPoint[1], box_center[2]);
	case 4: return Eigen::Vector3d(box_center[0], box_center[1], minPoint[2]);
	case 5: return Eigen::Vector3d(box_center[0], box_center[1], maxPoint[2]);
	}
}
Eigen::Vector3d  Box3::OwnClosestPoint(Line *line)
{    
     Eigen::Vector3d l_pt1[8];
     Eigen::Vector3d l_pt2[6];
     Eigen::Vector3d sideCenters[6];
     Eigen::Vector3d Corners[8];
    double  distance;
    double min_distance=1000;
    Eigen::Vector3d closest_point;
      for (int i = 0; i < 8; i++)
    {
           Corners[i]=this->CornerPoint(i); 
    }
      for (int i = 0; i < 6; i++)
    {
           sideCenters[i]=this->SideCenterPoint(i); 
    }
for (int i = 0; i < 8; i++)
{     
      Eigen::Vector3d c_pt = Corners[i];
      l_pt1[i] = line->getClosestPointToPoint(c_pt);
      distance = std::sqrt((std::pow(l_pt1[i][0]-c_pt[0],2)+std::pow(l_pt1[i][1]-c_pt[1],2)+std::pow(l_pt1[i][2]-c_pt[2],2))*1.0);
      if (distance<min_distance){
          min_distance=distance;
          closest_point= c_pt;
      }
}
for (size_t i = 0; i < 6; i++)
{
      Eigen::Vector3d c_pt = sideCenters[i];
      l_pt2[i] = line->getClosestPointToPoint(c_pt);
      distance= std::sqrt((std::pow(l_pt2[i][0]-c_pt[0],2)+std::pow(l_pt2[i][1]-c_pt[1],2)+std::pow(l_pt2[i][2]-c_pt[2],2))*1.0);
      if (distance<min_distance){
          min_distance=distance;
          closest_point= c_pt;
      }
}
 return closest_point;
}
/// Computes the closest point inside this AABB to the given point.
Eigen::Vector3d Box3::ClosestPoint(const Eigen::Vector3d &targetPoint)
		{
		  Eigen::Vector3d  point;	
		  float x = std::max(minPoint[0], std::min(targetPoint[0], maxPoint[0]));
		  float y = std::max(minPoint[1], std::min(targetPoint[1], maxPoint[1]));
		  float z = std::max(minPoint[2], std::min(targetPoint[2], maxPoint[2]));		
		  point[0]=x;
          point[1]=y;
          point[2]=z;
                   	  
		  return point;
		  
		}

   
 void Box3::getClosestPoints(Eigen::MatrixXd &closestPoints, Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
    // Straight line distance between closeset point on box to given point.
        this->getClosestPoints(closestPoints, capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getClosestPoints(closestPoints, sphere);
        }else{  
            Box3 *box = dynamic_cast<Box3*>(primitive);
        if(box){
            this->getClosestPoints(closestPoints, box);
		 
        }

	}
    }
}

	

		//return Max(0.f, Distance(sphere.pose) - sphere.r);
		
		
   void Box3::getClosestPoints(Eigen::MatrixXd &closestPoints, Capsule *capsule){
	//    
      Eigen::Vector3d basePoint, endPoint, ownClosestPoint, obstacleClosestPoint;
	 // here sphere.pose represents center of sphere
  // get box closest point to sphere center by clamping
      Eigen::Vector4d origin(0, 0, 0, 1);
      Eigen::Vector4d zDirectionCapsule(0, 0, capsule->getLength(), 1);


       basePoint = (capsule->pose * origin).head(3);
       endPoint  = (capsule->pose * zDirectionCapsule).head(3);
		  // this is the same as isPointInsideSphere
       Line axisOfSymmetryCapsule(basePoint, endPoint);
     
///  calculate distance between other shape primitives
       ownClosestPoint = (this->pose * origin).head(3);
        ownClosestPoint = this->OwnClosestPoint(&axisOfSymmetryCapsule);
    // Straight line distance between closeset point on box to given point.
        obstacleClosestPoint = axisOfSymmetryCapsule.getClosestPointToPoint(ownClosestPoint );
       closestPoints.row(0) = ownClosestPoint;
    closestPoints.row(1) = obstacleClosestPoint;
		 
	}
	
   
   void Box3::getClosestPoints(Eigen::MatrixXd &closestPoints, Sphere *sphere){
	
		//return Max(0.f, Distance(sphere.pose) - sphere.r);
		
		
			   // Find the point on this AABB closest to the sphere center.
	//    
     Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
	 // here sphere.pose represents center of sphere
  // get box closest point to sphere center by clamping
      Eigen::Vector4d origin(0, 0, 0, 1);

    obstacleClosestPoint = (sphere->pose * origin).head(3);
    ownClosestPoint = this->ClosestPoint(obstacleClosestPoint);

    closestPoints.row(0) = ownClosestPoint;
    closestPoints.row(1) = obstacleClosestPoint;
		  // this is the same as isPointInsideSphere


	}
   void Box3::getClosestPoints(Eigen::MatrixXd &closestPoints, Box3 *box){}
		
   
   void Box3::getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        this->getShortestDirection(shortestDirection, capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            this->getShortestDirection(shortestDirection, sphere);
        }else{Box3 *box = dynamic_cast<Box3*>(primitive);
        if(box){
            this->getShortestDirection(shortestDirection, box);
        }
	/// Checks for intersection between box and sphere
        }
    }
}
	   // Find the point on this AABB closest to the sphere center.

   void Box3::getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule){
      Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    
    this->getClosestPoints(closestPoints, capsule);
    ownClosestPoint = closestPoints.row(0);
    obstacleClosestPoint = closestPoints.row(1);
    shortestDirection = obstacleClosestPoint - ownClosestPoint;


   }
   void Box3::getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere){
		  // this is the same as isPointInsideSphere
   Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
    Eigen::MatrixXd closestPoints(2, 3);
    this->getClosestPoints(closestPoints, sphere);
    ownClosestPoint = closestPoints.row(0);
    obstacleClosestPoint = closestPoints.row(1);
    shortestDirection = obstacleClosestPoint - ownClosestPoint;
   }





   void Box3::getShortestDirection(Eigen::Vector3d &shortestDirection, Box3 *box){
}



   double Box3::getShortestDistance(Primitive *primitive){
    Capsule *capsule = dynamic_cast<Capsule*>(primitive);
    if(capsule){
        return this->getShortestDistance(capsule);
    }else{
        Sphere *sphere = dynamic_cast<Sphere*>(primitive);
        if(sphere){
            return this->getShortestDistance(sphere);
        }else{
            Box3 *box = dynamic_cast<Box3*>(primitive);
        if(box){
            return this->getShortestDistance(box);
        }

        }
    }
 }




   double Box3::getShortestDistance(Capsule *capsule){
        double shortestDistance = 0;
    Eigen::Vector3d shortestDirection;
    this->getShortestDirection(shortestDirection, capsule);
    shortestDistance = shortestDirection.norm() - capsule->getRadius() ;
    return shortestDistance;
}
 
   
   double Box3::getShortestDistance(Sphere *sphere){


    double shortestDistance = 0;
    Eigen::Vector3d shortestDirection;
    this->getShortestDirection(shortestDirection, sphere);

    shortestDistance = shortestDirection.norm() - sphere->getRadius();

    return shortestDistance;
// double getShortestDistance(Sphere *sphere);
//sdouble Mybox::getShortestDistance(Sphere &sphere){ return 0.0;}
   }
//Mybox Mybox::MinimalEnclosingAABB() const { return *this; }
   double Box3::getShortestDistance(Box3 *box){};
