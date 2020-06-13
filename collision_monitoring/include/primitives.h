#ifndef PRIMITIVES_H
#define PRIMITIVES_H

#include <tuple>
#include <vector>
#include <Eigen/Dense>


/** primitive.h
 * 
 * This file contains the definitions of all the primitive shapes. It includes
 * the classes to describe the geometry of the objects. 
 */

/* Class declarations */
class Line;
class Capsule;
class Sphere;
//class Cylinder;

/**
 * An abstract class that contains the basic attributes of all primitives
 * 
 * This abstract class gets inherited by all primitive shapes. All shapes 
 * have a pose expressed by a Eigen Matrix4d, and a family of methods
 * to get the shortest distrance from this primitive to another one depending
 * the class of the latter.
 */

class Primitive
{    
    public:
        /** Performs a dinamic cast to overload the distance functions
        * 
        * This method takes an object that inherits from primitive and
        * performs a dynamic cast to call the correct getShortestDistance
        * method depending on the class of the shape. returns a double value.
        *
        * @param primitive address of the primitive object.
        * @return the closest distance between this and the second primitive.
        */
        virtual double getShortestDistance(Primitive *primitive) = 0;

        /** Returns the distance between this primitive and a Capsule primitive
        * 
        * This method takes a capsule object and returns the closest distance
        * between this primitive and a capsule object. returns a double value.
        *
        * @param primitive address of the primitive object
        * @return the closest distance between the primitive and capsule
        */

        virtual double getShortestDistance(Capsule *capsule) = 0;
        
        /** Returns the distance between this primitive and a Sphere primitive
        * 
        * This method takes a capsule object and returns the closest distance
        * between this primitive and a sphere object. returns a double value.
        *
        * @param primitive address of the primitive object
        * @return the closest distance between the primitive and sphere
        */
        virtual double getShortestDistance(Sphere *sphere) = 0;

        /** Returns the distance between this primitive and a Cylinder primitive
        * 
        * This method takes a capsule object and returns the closest distance
        * between this primitive and a cylinder object. returns a double value.
        *
        * @param primitive address of the primitive object
        * @return the closest distance between the primitive and cylinder
        */
        //virtual double getShortestDistance(Cylinder *cylinder) = 0;

        Eigen::Matrix4d pose; /* pose of the primitive */

};

/**
 * Class to help calculate the closest distance between primitives.
 * 
 * This class describes a line using two points (represented as Eigen's 
 * Vector3D)
 */
class Line{
    private:
        Eigen::Vector3d basePoint; /* start point of the line */
        Eigen::Vector3d endPoint; /* end point of the line */

        /** one liner
        *
        * description
        * 
        * @param point a point space represented with a Vector3d\
        * @return A Vector3d object.
        */
        Eigen::Vector3d projectionPoint(Eigen::Vector3d point);

    public:
        /** Constructor of the Line class
        * 
        * @param basePoint start point of the line represented with a Vector3d.
        * @param endPoint end point of the line represented with a Vector3d.
        */
        Line(Eigen::Vector3d basePoint, Eigen::Vector3d endPoint);

        /* Destructor of the class Line */
        ~Line();

        /** Getter of the base point    
        *
        * @return A vector3d object with the base point.
        */
        Eigen::Vector3d getBasePoint();

        /** Getter of the end point    
        *
        * @return A vector3d object with the end point.
        */
        Eigen::Vector3d getEndPoint();

        /** one liner
        *
        * description
        * 
        * @param point a point space represented with a Vector3d\
        * @return A Vector3d object.
        */
        double getShortestDistanceToVertex(Eigen::Vector3d vertex);

        /** one liner
        *
        * description
        * 
        * @param point a point space represented with a Vector3d\
        * @return A Vector3d object.
        */
        double getShortestDistanceToLine(Line line);
        
};

/**
 * The Capsule class. A cylinder with two half spheres at both ends.
 * 
 * This class is a shape that inherits from primitive. It's similar to a 
 * cylinder containing half spheres in the ends, making a capsule.
 */
class Capsule: public Primitive{
    protected:
        float length; /* lenght of the capsule */
        float radius; /* radius of the capsule */
        
    public:
        /** Constructor of Capsule class
        * 
        * @param pose start point of the line represented with a Matrix4d.
        * @param length lenght of the capsule
        * @param radius radius of the capsule
        */
        Capsule(Eigen::Matrix4d pose, double length, double radius);

        /* Destructor of the class Capsule */
        ~Capsule();

        /** Getter of lenght
        *
        * @return the lenght of the capsule
        */
        float getLength();

        /** Getter of radius
        *
        * @return the radius of the capsule
        */
        float getRadius();


        double getShortestDistance(Primitive *primitive);
        double getShortestDistance(Capsule *capsule);
        double getShortestDistance(Sphere *sphere);
        // std::vector<double> getClosestPoint(std::vector<double>);
};

// class Cylinder: public Primitive{
//     protected:
//         float length;
//         float radius;

//     public:
//         Cylinder(Eigen::Matrix4d pose, double length, double radius);
//         ~Cylinder();

//         double getShortestDistance(Primitive *primitive);
//         double getShortestDistance(Capsule *capsule);
//         double getShortestDistance(Cylinder *cylinder);
//         double getShortestDistance(Sphere *sphere);
// };

class Sphere: public Primitive{
    private:
        float radius;
        
    public:
        /** Constructor of Capsule class
        * 
        * @param pose center point of the line represented with a Matrix4d.
        * @param radius radius of the sphere
        */
        Sphere(Eigen::Matrix4d pose, double radius);

        /* Destructor of the class Sphere */
        ~Sphere();

        /** Getter of radius
        *
        * @return the radius of the sphere
        */
        float getRadius();

        double getShortestDistance(Primitive *primitive);
        double getShortestDistance(Capsule *capsule);
        double getShortestDistance(Sphere *sphere);
        // std::vector<double> getClosestPoint(std::vector<double>);
};



#endif // PRIMITIVES_H