#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include <tuple>
#include <vector>
#include <Eigen/Dense>


class Primitive
{
    // private:

    //     // start with a point and then we can overload for other primitives
    //     // and then finally links
        

    //     // Same as above start with point then overload
    //     // virtual std::vector<double> getClosestPoint(std::vector<double>) = 0;
    
    public:
        virtual double getShortestDistance(Primitive *obstacle) = 0;
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
    private:
        float length;
        float radius;
        
    public:
        Cylinder(Eigen::Matrix4d pose, double length, double radius);
        ~Cylinder();
        double getShortestDistance(Primitive *obstacle);
        // std::vector<double> getClosestPoint(std::vector<double>);
};

// class N_ellipsoid: public Primitive {
//     private:
//         //Primitive parameters

//     public:
//         N_ellipsoid(std::vector<double> pose);
//         ~N_ellipsoid();
//         double getShortestDistance(N_ellipsoid *);
//         std::vector<double> getClosestPoint(std::vector<double>);
// };


#endif // PRIMITIVES_H