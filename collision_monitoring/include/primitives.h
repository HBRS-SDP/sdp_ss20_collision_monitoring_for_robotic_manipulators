#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include <tuple>
#include <vector>
#include <Eigen/Dense>


class Primitive
{
    protected:

        // start with a point and then we can overload for other primitives
        // and then finally links
        virtual double getShortestDistance(Primitive *obstacle) = 0;

        // Same as above start with point then overload
        // virtual std::vector<double> getClosestPoint(std::vector<double>) = 0;
    
    public:
        Eigen::Matrix4d pose;
        //virtual ~Primitive();

};

class Edge{
    private:
        Eigen::Vector3d basePoint;
        Eigen::Vector3d endPoint;
        Eigen::Vector3d projectionPoint(Eigen::Vector3d point);

    public:
        Edge(Eigen::Vector3d basePoint, Eigen::Vector3d endPoint);
        ~Edge();
        Eigen::Vector3d getBasePoint();
        Eigen::Vector3d getEndPoint();
        double getShortestDistanceVertex(Eigen::Vector3d vertex);
        double getShortestDistanceEdge(Edge edge);
        
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
//         //Primitve parameters

//     public:
//         N_ellipsoid(std::vector<double> pose);
//         ~N_ellipsoid();
//         double getShortestDistance(N_ellipsoid *);
//         std::vector<double> getClosestPoint(std::vector<double>);
// };


#endif // PRIMITIVES_H