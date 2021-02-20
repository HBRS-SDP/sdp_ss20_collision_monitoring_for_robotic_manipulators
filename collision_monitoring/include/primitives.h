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
class Mybox;
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
        
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive);
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule);
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere);

        double getShortestDistance(Primitive *primitive);
        double getShortestDistance(Capsule *capsule);
        double getShortestDistance(Sphere *sphere);
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

        void getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive);
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule);
        void getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere);

        double getShortestDistance(Primitive *primitive);
        double getShortestDistance(Capsule *capsule);
        double getShortestDistance(Sphere *sphere);
};


class Mybox{
	
public:

      
      /// Specifies center of box in world frame. 
     Eigen::Vector3d c;

     /// Specifies vector of how far the box extends on each axes.
     Eigen::Vector3d extents; //should be (0.66,0.60,0.30);  // These fixed coardinated have been extracted form narko base urdf file
    
    

		/// Specifies the minimum extent of this AABB in the world space x, y and z axes.
	  Eigen::Vector3d minPoint; 
	  
	  /// Specifies the maximum extent of this AABB in the world space x, y and z axes.
	  Eigen::Vector3d maxPoint;
	  
	 
	  /// Constructs this AABB by specifying the minimum and maximum extending corners of the box.
	
	Mybox(const Eigen::Vector3d &minPoint, const Eigen::Vector3d &maxPoint);
	 
	 
	 /** Constructor of Mybox class
        * 
        * @param    pose    center point of the box represented with a Matrix4d.
        * @param    axis  axis lenths along 3 axis
        */
       
      Mybox(Eigen::Vector3d &center);	 


      //Mybox(Eigen::Matrix4d pose, Eigen::Vector3d basePoint);	  


/// Returns the minimum world-space coordinate along the given axis.
	float MinX() const { return minPoint[0]; } ///  minimum bound along x-axis
	float MinY() const { return minPoint[1]; } ///  minimum bound along y-axis
	float MinZ() const { return minPoint[2]; } ///  minimum bound along z-axis
	/// Returns the maximum world-space coordinate along the given axis.
	float MaxX() const { return maxPoint[0]; } ///  maximum bound along x-axis
	float MaxY() const { return maxPoint[1]; } ///  maximum bound along y-axis
	float MaxZ() const { return maxPoint[2]; } ///  maximum bound along z-axis

/// Computes the closest point inside this AABB to the given point.
	Eigen::Vector3d ClosestPoint(const Eigen::Vector3d  &targetPoint)const;

/// Compute distance between this AABB and the given object.
    float Distance(const Sphere &sphere);
	
    float Distance(const Eigen::Vector3d &point);
	
/// Checks for intersection between objects	
	bool Intersects(const Mybox &aabb) ;
	bool Intersects(const Line &line) ;
	bool Intersects(const Capsule &capsule) ;
    bool Intersects( Sphere &sphere) const;
	// Mybox MinimalEnclosingAABB(const Eigen::Vector3d *pointArray, int numPoints);
    // static Mybox MinimalEnclosingAABB(const Eigen::Vector3d *pointArray, int numPoints);
	// Mybox MinimalEnclosingAABB() const { return *this; }
	// void SetNegativeInfinity();
	// void SetFrom(auto *pointArray, int numPoints);
	// Mybox Translated(const Eigen::Vector3d  &offset) const;
	void Enclose(Eigen::Vector3d &point);
	
    
    void getClosestPoints(Eigen::MatrixXd &closestPoints, Primitive *primitive);
    void getClosestPoints(Eigen::MatrixXd &closestPoints, Capsule *capsule);
    void getClosestPoints(Eigen::MatrixXd &closestPoints, Sphere *sphere);
    
    void getShortestDirection(Eigen::Vector3d &shortestDirection, Primitive *primitive);
    void getShortestDirection(Eigen::Vector3d &shortestDirection, Capsule *capsule);
    void getShortestDirection(Eigen::Vector3d &shortestDirection, Sphere *sphere);

    double getShortestDistance(Primitive *primitive);
    double getShortestDistance(Capsule *capsule);
    // double getShortestDistance(Sphere *sphere);
    double getShortestDistance(Sphere &sphere); 
    Mybox MinimalEnclosingAABB() const { return *this; }
    
	

};
#endif // PRIMITIVES_H