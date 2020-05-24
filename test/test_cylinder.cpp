#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include "primitives.h"
#include <iostream>
#include <Eigen/Dense>


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

    Cylinder *Link_1 = new Cylinder(pose_1, length_1, radius_1);
    Cylinder *Link_2 = new Cylinder(pose_2, length_2, radius_2);

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

TEST_CASE( "3rd test case lamda >= 1", "[cylinders]" ) {
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

TEST_CASE( "4th test case lamda <= 0", "[cylinders]" ) {
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