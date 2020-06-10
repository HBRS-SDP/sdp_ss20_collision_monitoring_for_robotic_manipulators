#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include <tuple>
#include <vector>
#include <Eigen/Dense>

class Line;
class Cylinder;
class Sphere;

class Primitive
{
    // private:

    //     // start with a point and then we can overload for other primitives
    //     // and then finally links
        

    //     // Same as above start with point then overload
    //     // virtual std::vector<double> getClosestPoint(std::vector<double>) = 0;
    
    public:
        virtual double getShortestDistance(Primitive *primitive) = 0;
        virtual double getShortestDistance(Cylinder *cylinder) = 0;
        virtual double getShortestDistance(Sphere *sphere) = 0;
        // virtual double getShortestDistance(Capsule capsule) = 0;
        Eigen::Matrix4d pose;
        //virtual ~Primitive();
};

class Line{
    private:
        Eigen::Vector3d basePoint;
        Eigen::Vector3d endPoint;
        Eigen::Vector3d projectionPoint(Eigen::Vector3d point);

    public:
        Line(Eigen::Vector3d basePoint, Eigen::Vector3d endPoint);
        ~Line();
        Eigen::Vector3d getBasePoint();
        Eigen::Vector3d getEndPoint();
        double getShortestDistanceToVertex(Eigen::Vector3d vertex);
        double getShortestDistanceToLine(Line line);
        
};

class Cylinder: public Primitive{
    protected:
        float length;
        float radius;
        
    public:
        Cylinder(Eigen::Matrix4d pose, double length, double radius);
        ~Cylinder();

        float getLength();
        float getRadius();

        double getShortestDistance(Primitive *primitive);
        double getShortestDistance(Cylinder *cylinder);
        double getShortestDistance(Sphere *sphere);
        // std::vector<double> getClosestPoint(std::vector<double>);
};

// class Capsule: public Cylinder{

// };

class Sphere: public Primitive{
    private:
        float radius;
        
    public:
        Sphere(Eigen::Matrix4d pose, double radius);
        ~Sphere();

        float getRadius();

        double getShortestDistance(Primitive *primitive);
        double getShortestDistance(Cylinder *cylinder);
        double getShortestDistance(Sphere *sphere);
        // std::vector<double> getClosestPoint(std::vector<double>);
};



#endif // PRIMITIVES_H