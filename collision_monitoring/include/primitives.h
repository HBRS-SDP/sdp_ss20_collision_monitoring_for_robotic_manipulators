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
class Box3;
class Ray;
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
        /** Performs a dynamic cast to overload the direction functions
        * 
        * This method takes an object that inherits from primitive and
        * performs a dynamic cast to call the correct getClosestPoints
        * method depending on the class of the shape.
        *
        * @param        primitive       address of the primitive object.
        * @param[out]   closestPoints   the closest points in this primitive and primitive
        */
        virtual void getClosestPoints(Eigen::MatrixXd &closestPoints, Primitive *primitive) = 0;

        /** Finds the shortest distance between this primitive and a Capsule primitive
        * 
        * This method takes a capsule object and returns the closest points
        * in this primitive and in a capsule object.
        *
        * @param        capsule         address of the primitive object
        * @param[out]   closestPoints   the closest points in this primitive and capsule
        */

        virtual void getClosestPoints(Eigen::MatrixXd &closestPoints, Capsule *capsule) = 0;
        
        /** Finds the shortest distance between this primitive and a Sphere primitive
        * 
        * This method takes a capsule object and returns the closest points
        * in this primitive and in a sphere object.
        *
        * @param        sphere          address of the primitive object
        * @param[out]   closestPoints   the closest points in this primitive and sphere
        */
        virtual void getClosestPoints(Eigen::MatrixXd &closestPoints, Sphere *sphere) = 0;
        
	/** Finds the shortest distance between this primitive and a Box primitive
        * 
        * This method takes a box object and returns the closest points
        * in this primitive and in a box object.
        *
        * @param        box          address of the primitive object
        * @param[out]   closestPoints   the closest points in this primitive and box
        */

         virtual void getClosestPoints(Eigen::MatrixXd &closestPoints, Box3 *box) = 0;

        /** Performs a dynamic cast to overload the direction functions
        * 
        * This method takes an object that inherits from primitive and
        * performs a dynamic cast to call the correct getShortestDirection
        * method depending on the class of the shape.
        *
        * @param        primitive           address of the primitive object.
        * @param[out]   shortestDirection   a vector in 3D that represents the closes direction between this and the second primitive.
        */
        virtual void getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive) = 0;

        /** Finds the shortest distance between this primitive and a Capsule primitive
        * 
        * This method takes a capsule object and returns the closest direction
        * between this primitive and a capsule object.
        *
        * @param        capsule             address of the primitive object
        * @param[out]   shortestDirection   a vector in 3D that represents the closes direction between the primitive and capsule
        */

        virtual void getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule) = 0;
        
        /** Finds the shortest distance between this primitive and a Sphere primitive
        * 
        * This method takes a capsule object and returns the closest direction
        * between this primitive and a sphere object.
        *
        * @param        sphere              address of the primitive object
        * @param[out]   shortestDirection   a vector in 3D that represents the closes direction between the primitive and sphere
        */
        virtual void getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere) = 0;


	/** Finds the shortest direction between this primitive and a Box primitive
        * 
        * This method takes a box object and returns the closest direction
        * between this primitive and a box object.
        *
        * @param        box              address of the primitive object
        * @param[out]   shortestDirection   a vector in 3D that represents the closes direction between the primitive and box
        */
        virtual void getShortestDirection(Eigen::Vector3d &shortestDirection, Box3 *box) = 0;

        /** Performs a dynamic cast to overload the distance functions
        * 
        * This method takes an object that inherits from primitive and
        * performs a dynamic cast to call the correct getShortestDistance
        * method depending on the class of the shape. returns a double value.
        *
        * @param    primitive   address of the primitive object.
        * @return   the closest distance between this and the second primitive.
        */
        virtual double getShortestDistance(Primitive *primitive) = 0;

        /** Finds the shortest distance between this primitive and a Capsule primitive
        * 
        * This method takes a capsule object and returns the closest distance
        * between this primitive and a capsule object. returns a double value.
        *
        * @param    capsule     address of the primitive object
        * @return   the closest distance between the primitive and capsule
        */

        virtual double getShortestDistance(Capsule *capsule) = 0;
        
        /** Finds the shortest distance between this primitive and a Sphere primitive
        * 
        * This method takes a capsule object and returns the closest distance
        * between this primitive and a sphere object. returns a double value.
        *
        * @param    sphere      address of the primitive object
        * @return   the closest distance between the primitive and sphere
        */
        virtual double getShortestDistance(Sphere *sphere) = 0;


	 /** Finds the shortest distance between this primitive and a Box primitive
        * 
        * This method takes a capsule object and returns the closest distance
        * between this primitive and a box object. returns a double value.
        *
        * @param    box      address of the primitive object
        * @return   the closest distance between the primitive and box
        */
        virtual double getShortestDistance(Box3 *box) = 0;

        /** Finds the shortest distance between this primitive and a Cylinder primitive
        * 
        * This method takes a capsule object and returns the closest distance
        * between this primitive and a cylinder object. returns a double value.
        *
        * @param    cylinder    address of the primitive object
        * @return   the closest distance between the primitive and cylinder
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

        /** Projects a point onto the xy-plane of the line
        *
        * The reference frame of the line is on the midpoint of the line
        * with the z-axis in direction of the normal of the line.
        * This method projects a point onto the xy-plane of the
        * reference frame of the line and return the resulting point.
        * 
        * @param    point   a point space represented with a Vector3d\
        * @return   the projected point
        */
        Eigen::Vector3d projectionPoint(Eigen::Vector3d point);

    public:
        /** Constructor of the Line class
        * 
        * @param    basePoint   start point of the line represented with a Vector3d.
        * @param    endPoint    end point of the line represented with a Vector3d.
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

        /** Finds the closest point on this Line to a point
        *
        * This method takes a point and returns the closest point
        * on this Line and the given point. returns a Vector3d.
        * 
        * @param    point   a point space represented with a Vector3d\
        * @return   the closest point on this Line to point
        */
        Eigen::Vector3d getClosestPointToPoint(Eigen::Vector3d point);

        /** Finds the closest points on this Line and on another Line
        *
        * This method takes a point and returns the closest point
        * on this Line and the given line. returns a Vector3d.
        * 
        * @param        line            a line represented with a Vector3d.
        * @param[out]   closestPoints   the closests points on this line and on line
        */
        void getClosestPointsBetweenLines(Eigen::MatrixXd &closestPoints, Line line);

        /** Finds the shortest distance between this Line and a point
        *
        * This method takes a point and returns the closest distance
        * between this Line and the given point. returns a double value.
        * 
        * @param     point           a point space represented with a Vector3d.
        * @return The shortest distance from the point to the obstacle
        */
        double getShortestDistanceToPoint(Eigen::Vector3d point);

        /** Finds the shortest distance between this Line and another Line
        *
        * This method takes a Line and returns the closest distance
        * between this Line and the given line. returns a double value.
        * 
        * @param    line    a line represented with a Vector3d\
        * @return   the closest distance between the Line and line
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
        /// length of the capsule
        float length;
        /// radius of the capsule
        float radius;
        
    public:
        /** Constructor of Capsule class
        * 
        * @param pose start point of the line represented with a Matrix4d.
        * @param length  length of the capsule
        * @param radius  radius of the capsule
        */
        Capsule(Eigen::Matrix4d pose, double length, double radius);

        /** Copy onstructor of Capsule class
        * 
        * @param capsule the capsule instance to copy
        */
        Capsule(Capsule* capsule);
        /* Destructor of the class Capsule */
        ~Capsule();

        /** Getter of length
        *
        * @return the length of the capsule
        */
        float getLength();

        /** Getter of radius
        *
        * @return the radius of the capsule
        */
        float getRadius();

        void getClosestPoints(Eigen::MatrixXd &closestPoints, Primitive *primitive);
        void getClosestPoints(Eigen::MatrixXd &closestPoints, Capsule *capsule);
        void getClosestPoints(Eigen::MatrixXd &closestPoints, Sphere *sphere);
        void getClosestPoints(Eigen::MatrixXd &closestPoints, Box3 *box);
        
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive);
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule);
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere);
		void getShortestDirection(Eigen::Vector3d &shortestDirection, Box3 *box);

        double getShortestDistance(Primitive *primitive);
        double getShortestDistance(Capsule *capsule);
        double getShortestDistance(Sphere *sphere);
		double getShortestDistance(Box3 *box);
};

/**
 * The Sphere class
 * 
 * This class is a shape that inherits from primitive.
 */
class Sphere: public Primitive{
    private:
        float radius;
        
    public:
        /** Constructor of Capsule class
        * 
        * @param    pose    center point of the line represented with a Matrix4d.
        * @param    radius  radius of the sphere
        */
        Sphere(Eigen::Matrix4d pose, double radius);

        /** Copy costructor of capsule class
         * 
         * @param sphere The Sphere instance to copy
         */
        Sphere(Sphere* sphere);

        /* Destructor of the class Sphere */
        ~Sphere();

        /** Getter of radius
        *
        * @return the radius of the sphere
        */
        float getRadius();

        void getClosestPoints(Eigen::MatrixXd &closestPoints, Primitive *primitive);
        void getClosestPoints(Eigen::MatrixXd &closestPoints, Capsule *capsule);
        void getClosestPoints(Eigen::MatrixXd &closestPoints, Sphere *sphere);
        void getClosestPoints(Eigen::MatrixXd &closestPoints, Box3 *box);

        void getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive);
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule);
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere);
		void getShortestDirection(Eigen::Vector3d &shortestDirection, Box3 *box);

        double getShortestDistance(Primitive *primitive);
        double getShortestDistance(Capsule *capsule);
        double getShortestDistance(Sphere *sphere);
		double getShortestDistance(Box3 *box);
};


/**
 * The Ray class
 * 
 * This class is a shape that describes the Ray in 3D space.
 */

class Ray 
{ 
public: 

    Ray(const Eigen::Vector3d orig, const Eigen::Vector3d &dir); 
    
    Eigen::Vector3d orig; 
    Eigen::Vector3d dir;       // ray orig and dir 
    Eigen::Vector3d invdir; 
    int sign[3]; 

};


/**
 * The Box class
 * 
 * This class is a shape that describes the base of the robot as well as obstacles of the shape of a 3D box (cube).
 */


class Box3:public Primitive{ 
public: 

    //axis bound limit of 3D box
    Eigen::Vector3d bounds[2]; 

    //Minimum axis bound of 3D Box
    Eigen::Vector3d minPoint= bounds[0];

    //Maximum axis bound of 3D Box.
    Eigen::Vector3d maxPoint= bounds[1];

    //Center position of 3D Box.
    Eigen::Vector3d box_center;

    //Describes how far apart the minimum and maximum axis bounds are from the center of the box.
    Eigen::Vector3d extents;
	   
     
    /** Constructor of Box class
        * 
        * @param    center center point of the 3D Box represented with a Vector3d.
        */
    Box3(Eigen::Vector3d &center);

    /** Constructor of Box class
        * 
        * @param    vmin minimum axis bound of 3D Box represented with a Vector3d.
        * @param    vmax maximum axis bound of 3D Box represented with a Vector3d.
        */
    Box3(const Eigen::Vector3d &vmin, const Eigen::Vector3d &vmax); 

    /** Constructor of Box class
        * 
        * @param    box an object of the class box
        * s
        */

    Box3(Box3* box);

    
    Box3(Eigen::Matrix4d &pose, double x,double y,double z);
    Box3(Eigen::Vector3d &pose, double x,double y,double z);




    /* Destructor of the class Box */
    ~Box3();
    
   
    
   /** Function to check intersection of box with the ray.
        * 
        * @param    r object of class Ray.
        * @return true or false based on intersection.
        */
   bool intersection ( const Ray &r) const; 

   /** Function to check intersection of box with the ray.
        * 
        * @param edgeIndex    index of the edge.
        * @return edge of the box as a line object.
        */
   Line* Edge(int edgeIndex) const; 

   /** Function to check intersection of box with the ray.
        * 
        * @return vector filled with all the edges of the box.
        */ 
   std::vector<Line*>  EdgeList() const;

   /** Function to check intersection of box with the ray.
        * 
        * @param    cornerIndex index of the corner of the box.
        * @return corner of the box as a Vector3d.
        */
   Eigen::Vector3d CornerPoint(int cornerIndex) const;

   /** Function to check intersection of box with the ray.
        * 
        * @param    sideIndex index of the face/side of the box.
        * @return midpoint of the center of the side of the box as a Vector3d.
        */
   Eigen::Vector3d SideCenterPoint(int sideIndex) const;

   /** Function to check intersection of box with the ray
        * 
        * @param    line line Class object
        * @return closest point on the box to the line obstacle
        */
   Eigen::Vector3d OwnClosestPoint(Line *line);

   /** Function to check intersection of box with the ray
        * 
        * @param    sphere sphere Class object
        * @return closest point on the box to the sphere obstacle
        */
   Eigen::Vector3d OwnClosestPoint(Sphere *sphere);

 
   /** Function to get the closest point inside the box (AABB ) to an external point
        * 
        * @param targetPoint An external point to Box
        * @return closest point inside the box to the external point 
        */

   Eigen::Vector3d ClosestPoint(const Eigen::Vector3d &targetPoint);




   void getClosestPoints(Eigen::MatrixXd &closestPoints, Primitive *primitive);
   void getClosestPoints(Eigen::MatrixXd &closestPoints, Capsule *capsule);
   void getClosestPoints(Eigen::MatrixXd &closestPoints, Sphere *sphere);
   void getClosestPoints(Eigen::MatrixXd &closestPoints, Box3 *box);
        
   void getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive);
   void getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule);
   void getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere);
   void getShortestDirection(Eigen::Vector3d &shortestDirection, Box3 *box);


   double getShortestDistance(Primitive *primitive);
   double getShortestDistance(Capsule *capsule);
   double getShortestDistance(Sphere *sphere);
   double getShortestDistance(Box3 *box);
   
};
#endif // PRIMITIVES_H