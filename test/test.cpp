// Pulled from https://github.com/catchorg/Catch2/blob/master/docs/tutorial.md

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include <vector>
#include <Eigen/Core>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <libgen.h>


#define private public
#include "kinova_arm.h"
#include "primitives.h"
#include "monitor.h"
#include "arm.h"

double deg2rad(double v) {
    return v / 180 * M_PI;
}

std::string urdf_filename = "../urdf/GEN3_URDF_V12.urdf";

TEST_CASE( "1st test case", "[cylinders]" ) {
    double radius_1 = 10;
    double length_1 = 70.710678118654755;
    Eigen::Matrix4d pose_1;
    pose_1 << 0.89455844, -0.14058875,  0.42426407,  0,
                -0.14058875,  0.81254834,  0.56568542,  0,
                -0.42426407, -0.56568542,  0.70710678,  0,
                0,          0,          0,          1;

    double radius_2 = 10;
    double length_2 = 81.2403840463596;
    Eigen::Matrix4d pose_2;
    pose_2 << 0.76552284, -0.18758173, -0.61545745,  50,
                -0.18758173,  0.84993462, -0.49236596,  0,
                0.61545745,  0.49236596,  0.61545745,  0,
                0,          0,          0,          1;

    Primitive *Link_1 = new Cylinder(pose_1, length_1, radius_1);
    Primitive *Link_2 = new Cylinder(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(15.007002100700248).margin(0.001) );
}

TEST_CASE( "2nd test case", "[cylinders]" ) {
    double radius_1 = 10;
    double length_1 = 95.39392014169457;
    Eigen::Matrix4d pose_1;
    pose_1 <<   0.94911072,  -0.01696309,  -0.31448545,  40,
                -0.01696309,   0.99434564,  -0.10482848, -20,
                0.31448545,   0.10482848,   0.94345635, -30,
                0,           0,           0,           1;

    double radius_2 = 15;
    double length_2 = 70.0;
    Eigen::Matrix4d pose_2;
    pose_2 << 0.48571429,   0.17142857,  -0.85714286,  70,
                0.17142857,   0.94285714,   0.28571429, -10,
                0.85714286,  -0.28571429,   0.42857143,  30,
                0,           0,           0,           1;

    Cylinder *Link_1 = new Cylinder(pose_1, length_1, radius_1);
    Cylinder *Link_2 = new Cylinder(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(10.233213170882209).margin(0.001) );
}

TEST_CASE( "3rd test case lambda >= 1", "[cylinders]" ) {
    double radius_1 = 10;
    double length_1 = 95.39392014169457;
    Eigen::Matrix4d pose_1;
    pose_1 << 0.94911072,  -0.01696309,  -0.31448545,  40,
                -0.01696309,   0.99434564,  -0.10482848, -20,
                0.31448545,   0.10482848,   0.94345635, -30,
                0,           0,           0,           1;

    double radius_2 = 15;
    double length_2 = 78.74007874011811;
    Eigen::Matrix4d pose_2;
    pose_2 << 0.42771733, -0.16350933, -0.88900089, 80,
                -0.16350933,  0.95328305, -0.25400025, 30,
                0.88900089,  0.25400025,  0.38100038, 30,
                0,          0,          0,          1;

    Cylinder *Link_1 = new Cylinder(pose_1, length_1, radius_1);
    Cylinder *Link_2 = new Cylinder(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(14.779612647907754).margin(0.001) );
}

TEST_CASE( "4th test case lambda <= 0", "[cylinders]" ) {
    double radius_1 = 10;
    double length_1 = 70.0;
    Eigen::Matrix4d pose_1;
    pose_1 << 0.42857143, -0.28571429, -0.85714286, 70,
                -0.28571429,  0.85714286, -0.42857143,  0,
                0.85714286,  0.42857143,  0.28571429, 40,
                0,          0,          0,          1;

    double radius_2 = 15;
    double length_2 = 78.74007874011811;
    Eigen::Matrix4d pose_2;
    pose_2 << 0.42771733, -0.16350933, -0.88900089, 80,
                -0.16350933,  0.95328305, -0.25400025, 30,
                0.88900089,  0.25400025,  0.38100038, 30,
                0,          0,          0,          1;

    Cylinder *Link_1 = new Cylinder(pose_1, length_1, radius_1);
    Cylinder *Link_2 = new Cylinder(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(-2.4123024273687186).margin(0.001) );
}

TEST_CASE("Kinova_arm distance to obstacle", "[monitor]") {

    /*
    * THIS IS AN INCOMPLETE TEST CASE 
    */

    KinovaArm kinovaArm(urdf_filename);
    std::vector<double> testPose = {deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(30),
                                    deg2rad(30), deg2rad(30), deg2rad(30)};
    kinovaArm.updatePose(testPose);

    double radius_1 = 10;
    double length_1 = 70.710678118654755;

    Eigen::Matrix4d pose_1;
    pose_1 << 0.89455844, -0.14058875,  0.42426407,  0,
                -0.14058875,  0.81254834,  0.56568542,  0,
                -0.42426407, -0.56568542,  0.70710678,  0,
                0,          0,          0,          1;

    Cylinder Link_1(pose_1, length_1, radius_1);

    Monitor monitor(&kinovaArm);
    monitor.addObstacle(&Link_1);

    std::vector<std::vector<double>> distancesToObstacles = monitor.monitorCollisionWithObjects();
    std::vector<std::vector<double>> distancesToLinks = monitor.monitorCollisionWithArm();

    REQUIRE( 10 > 0.1 );    
}

TEST_CASE("Kinova_arm init", "[arm]") {
    KinovaArm kinovaArm(urdf_filename);
}

TEST_CASE("Kinova_arm destructor", "[arm]") {
    KinovaArm* kinovaArm = new KinovaArm(urdf_filename);
    delete(kinovaArm);
}

TEST_CASE("Kinova_arm set position", "[arm]") {
    KinovaArm kinovaArm(urdf_filename);
    std::vector<double> testPose = {deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(30),
                                    deg2rad(30), deg2rad(30), deg2rad(30)};
    kinovaArm.updatePose(testPose);
    Eigen::Matrix4d link2Pose;
    Eigen::Matrix4d endLinkPose;
    link2Pose << 0.43092, 0, 0.417668, -0.0026875,
                   0.376225, 1, -0.276125, -0.00465583,
                   -0.417668, 0, 0.43092, 0.28481,
                   0, 0, 0, 1;
    endLinkPose << 0.565619, 0, 0.573497, 0.285814,
                0.470212, 1, -0.620803, -0.251506,
                -0.573497, 0, 0.565619, 0.763581,
                0, 0, 0, 1;

    REQUIRE( fabs((link2Pose - kinovaArm.links[2]->pose).norm()) < 0.1);
    REQUIRE( fabs((endLinkPose - kinovaArm.links[5]->pose).norm()) < 0.1);
}

TEST_CASE("Kinova_arm test link positions", "[arm]") {
    KinovaArm kinovaArm(urdf_filename);
    std::vector<double> testPose = {deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(30),
                                    deg2rad(30), deg2rad(30), deg2rad(30)};
    kinovaArm.updatePose(testPose);

    Eigen::Vector4d origin(0, 0, 0, 1);

    for(int i=0; i < kinovaArm.nJoints-1; i++) {
        // get endpoints from the frames calculation
        Eigen::Matrix4d baseMatLink = kinovaArm.frameToMatrix(*kinovaArm.poses[i]);
        Eigen::Matrix4d endMatLink = kinovaArm.frameToMatrix(*kinovaArm.poses[i+1]);
        Eigen::Vector3d basePointLink = (baseMatLink * origin).head(3);
        Eigen::Vector3d endPointLink = (endMatLink * origin).head(3);

        // Get the pose from these points using the same method in kinovaArm
        Eigen::Matrix4d pose = kinovaArm.linkFramesToPose(*kinovaArm.poses[i], *kinovaArm.poses[i+1]);

        // get endpoints from the pose calculation
        Eigen::Vector4d zDirectionObstacle(0, 0, kinovaArm.lengths[i], 1);
        Eigen::Vector3d basePointLine = (pose * origin).head(3);
        Eigen::Vector3d endPointLine  = (pose * zDirectionObstacle).head(3);

        // compare using the two methods
        REQUIRE( fabs((basePointLine - basePointLink).norm()) < 0.1);
        REQUIRE( fabs((endPointLine - endPointLink).norm()) < 0.1);
    }
}