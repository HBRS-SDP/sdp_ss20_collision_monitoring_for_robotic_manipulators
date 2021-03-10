// Pulled from https://github.com/catchorg/Catch2/blob/master/docs/tutorial.md

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

// the debug var to turn verbose on and off
// #define DEBUG

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

TEST_CASE( "Parallel capsules", "[Capsule - Capsule]" ) {
    Eigen::Vector3d shortestDirection;
    double radius_1 = 10;
    double length_1 = 15;
    Eigen::Matrix4d pose_1;
    pose_1 << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    double radius_2 = 10;
    double length_2 = 15;
    Eigen::Matrix4d pose_2;
    pose_2 << 1, 0, 0, 40,
              0, 1, 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    Primitive *Link_1 = new Capsule(pose_1, length_1, radius_1);
    Primitive *Link_2 = new Capsule(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(20).margin(0.001) );
    REQUIRE( Link_2->getShortestDistance(Link_1) == Approx(20).margin(0.001) );

    delete Link_1;
    delete Link_2;
}

TEST_CASE( "1st test case", "[Capsule - Capsule]" ) {
    Eigen::Vector3d shortestDirection;
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

    Primitive *Link_1 = new Capsule(pose_1, length_1, radius_1);
    Primitive *Link_2 = new Capsule(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(15.007002100700248).margin(0.001) );
    REQUIRE( Link_2->getShortestDistance(Link_1) == Approx(15.007002100700248).margin(0.001) );

    delete Link_1;
    delete Link_2;
}

TEST_CASE( "2nd test case", "[Capsule - Capsule]" ) {
    Eigen::Vector3d shortestDirection;
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

    Primitive *Link_1 = new Capsule(pose_1, length_1, radius_1);
    Primitive *Link_2 = new Capsule(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(10.233213170882209).margin(0.001) );
    REQUIRE( Link_2->getShortestDistance(Link_1) == Approx(10.233213170882209).margin(0.001) );

    delete Link_1;
    delete Link_2;
}

TEST_CASE( "3rd test case lambda >= 1", "[Capsule - Capsule]" ) {
    Eigen::Vector3d shortestDirection;
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

    Primitive *Link_1 = new Capsule(pose_1, length_1, radius_1);
    Primitive *Link_2 = new Capsule(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(14.779612647907754).margin(0.001) );
    REQUIRE( Link_2->getShortestDistance(Link_1) == Approx(14.779612647907754).margin(0.001) );

    delete Link_1;
    delete Link_2;
}

TEST_CASE( "4th test case lambda <= 0", "[Capsule - Capsule]" ) {
    Eigen::Vector3d shortestDirection;
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

    Primitive *Link_1 = new Capsule(pose_1, length_1, radius_1);
    Primitive *Link_2 = new Capsule(pose_2, length_2, radius_2);

    REQUIRE( Link_1->getShortestDistance(Link_2) == Approx(1.21253086204284).margin(0.001) );
    REQUIRE( Link_2->getShortestDistance(Link_1) == Approx(1.21253086204284).margin(0.001) );

    delete Link_1;
    delete Link_2;
}


TEST_CASE( "1st test case sphere", "[Sphere - Sphere]" ) {
    double radius_1 = 20;
    Eigen::Matrix4d pose_1;
    pose_1 << 0.51701611,  -0.33046266,   0.78961305, -70,
                -0.33046266,   0.77389397,   0.54026156,   5,
                -0.78961305, -0.54026156,   0.29091007, -15,
                0,           0,           0,           1;

    double radius_2 = 30;
    Eigen::Matrix4d pose_2;
    pose_2 << 0.12071974, -0.60161281, -0.78961305, 25,
                -0.60161281,  0.58837018, -0.54026156, 70,
                0.78961305,  0.54026156, -0.29091007, 20,
                0,          0,          0,          1;

    Primitive *Sphere_1 = new Sphere(pose_1, radius_1);
    Primitive *Sphere_2 = new Sphere(pose_2, radius_2);

    REQUIRE( Sphere_1->getShortestDistance(Sphere_2) == Approx(70.3).margin(0.1) );

    delete Sphere_1;
    delete Sphere_2;
}

TEST_CASE( "2nd test case sphere", "[Sphere - Capsule]" ) {
    double radius_1 = 12;
    Eigen::Matrix4d pose_1;
    pose_1 << -0.41648471, -0.22511371, -0.8808316,  11.8,
                -0.22511371,  0.96422398, -0.13998546, 18,
                0.8808316,   0.13998546, -0.45226072, 27,
                0,          0,          0,          1;

    double radius_2 = 10;
    double length_2 = 62.10475022089695;
    Eigen::Matrix4d pose_2;
    pose_2 << 0.66072227,   0.19602713,  -0.72458226,  60,
                0.19602713,   0.88673988,   0.41864753, -16,
                0.72458226,  -0.41864753,   0.54746215,  30,
                0,           0,           0,           1;

    Primitive *Sphere_1 = new Sphere(pose_1, radius_1);
    Primitive *Link_2 = new Capsule(pose_2, length_2, radius_2);

    REQUIRE( Sphere_1->getShortestDistance(Link_2) == Approx(13.1).margin(0.1) );
    REQUIRE( Link_2->getShortestDistance(Sphere_1) == Approx(13.1).margin(0.1) );

    delete Link_2;
    delete Sphere_1;
}


TEST_CASE( "3rd test case sphere", "[Sphere - Capsule]" ) {
    double radius_1 = 12;
    Eigen::Matrix4d pose_1;
    pose_1 << -0.04687922,  -0.69791948,  -0.71464029, -10,
                -0.69791948,   0.53472035,  -0.47642686,  45,
                0.71464029,   0.47642686,  -0.51215887,  28,
                0,           0,           0,           1;

    double radius_2 = 10;
    double length_2 = 62.10475022089695;
    Eigen::Matrix4d pose_2;
    pose_2 << 0.66072227,   0.19602713,  -0.72458226,  60,
                0.19602713,   0.88673988,   0.41864753, -16,
                0.72458226,  -0.41864753,   0.54746215,  30,
                0,           0,           0,           1;

    Primitive *Sphere_1 = new Sphere(pose_1, radius_1);
    Primitive *Link_2 = new Capsule(pose_2, length_2, radius_2);

    REQUIRE( Sphere_1->getShortestDistance(Link_2) == Approx(34.1).margin(0.1) );
    REQUIRE( Link_2->getShortestDistance(Sphere_1) == Approx(34.1).margin(0.1) );

    delete Link_2;
    delete Sphere_1;
}

TEST_CASE( "4th test case sphere", "[Sphere - Capsule]" ) {
    double radius_1 = 12;
    Eigen::Matrix4d pose_1;
    pose_1 << -0.21308182,   0.24106113,  -0.94682927,  86,
                0.24106113,   0.95209683,   0.18815197, -26,
                0.94682927,  -0.18815197,  -0.26098499,  28,
                0,           0,           0,           1;     
     

    double radius_2 = 10;
    double length_2 = 62.10475022089695;
    Eigen::Matrix4d pose_2;
    pose_2 << 0.66072227,   0.19602713,  -0.72458226,  60,
                0.19602713,   0.88673988,   0.41864753, -16,
                0.72458226,  -0.41864753,   0.54746215,  30,
                0,           0,           0,           1;

    Primitive *Sphere_1 = new Sphere(pose_1, radius_1);
    Primitive *Link_2 = new Capsule(pose_2, length_2, radius_2);

    //std::cout << Sphere_1->getShortestDistance(Link_2) << std::endl;
    //std::cout << Link_2->getShortestDistance(Sphere_1) << std::endl;

    REQUIRE( Sphere_1->getShortestDistance(Link_2) == Approx(5.9).margin(0.1) );
    REQUIRE( Link_2->getShortestDistance(Sphere_1) == Approx(5.9).margin(0.1) );

    delete Link_2;
    delete Sphere_1;
}


// TEST_CASE("Kinova_arm init", "[arm]") {
//     KinovaArm kinovaArm(urdf_filename);
// }

// TEST_CASE("Kinova_arm destructor", "[arm]") {
//     KinovaArm* kinovaArm = new KinovaArm(urdf_filename);
//     delete(kinovaArm);
// }

// TEST_CASE("Kinova_arm init with base transform", "[arm]") {
//     Eigen::Matrix4d basePosition;
//     basePosition << 1, 0, 0, 0.1,
//                     0, 1, 0, 0.2,
//                     0, 0, 1, 0.4,
//                     0, 0, 0, 1;
//     KinovaArm kinovaArm(urdf_filename, basePosition);
// }

// TEST_CASE("Kinova_arm base transform destructor", "[arm]") {
//     Eigen::Matrix4d basePosition;
//     basePosition << 1, 0, 0, 0.1,
//                     0, 1, 0, 0.2,
//                     0, 0, 1, 0.4,
//                     0, 0, 0, 1;
//     KinovaArm* kinovaArm = new KinovaArm(urdf_filename, basePosition);
//     delete(kinovaArm);
// }

// TEST_CASE("Kinova_arm set position", "[arm]") {
//     KinovaArm kinovaArm(urdf_filename);
//     std::vector<double> testPose = {deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(30),
//                                    deg2rad(30), deg2rad(30), deg2rad(30)};
//     kinovaArm.updatePose(testPose);
//     Eigen::Matrix4d link2Pose;
//     Eigen::Matrix4d endLinkPose;
//     link2Pose << 1, 0, 0.417668, -0.0026875,
//                  0, 1.15523,-0.276125, -0.00465583,
//                  0, 0, 0.865626, 0.28481,
//                    0, 0, 0, 1;
//     endLinkPose <<  1, 0, 0.573497, 0.285814,
//                     0, 1.87086, -0.620803, -0.251506,
//                     0, 0, 0.534514, 0.763581,
//                     0, 0, 0, 1;

//     REQUIRE( fabs((link2Pose - kinovaArm.links[2]->pose).norm()) < 0.1);
//     REQUIRE( fabs((endLinkPose - kinovaArm.links[5]->pose).norm()) < 0.1);
// }

// TEST_CASE("Kinova_arm test link positions", "[arm]") {
//     KinovaArm kinovaArm(urdf_filename);
//     std::vector<double> testPose = {deg2rad(90), deg2rad(90), deg2rad(90), deg2rad(90),
//                                     deg2rad(90), deg2rad(90), deg2rad(90)};
//     kinovaArm.updatePose(testPose);

//     Eigen::Vector4d origin(0, 0, 0, 1);

//     for(int i=0; i < kinovaArm.nLinks; i++) {
//         // get endpoints from the frames calculation
//         Eigen::Matrix4d baseMatLink = kinovaArm.frameToMatrix(*kinovaArm.localPoses[i]);
//         Eigen::Matrix4d endMatLink = kinovaArm.frameToMatrix(*kinovaArm.localPoses[i+1]);
//         Eigen::Vector3d basePointLink = (baseMatLink * origin).head(3);
//         Eigen::Vector3d endPointLink = (endMatLink * origin).head(3);

//         // Get the pose from these points using the same method in kinovaArm
//         Eigen::Matrix4d pose = kinovaArm.linkFramesToPose(*kinovaArm.localPoses[i], *kinovaArm.localPoses[i+1]);


//         // get endpoints from the pose calculation
//         Eigen::Vector4d zDirectionObstacle(0, 0, kinovaArm.lengths[i], 1);
//         Eigen::Vector3d basePointLine = (pose * origin).head(3);
//         Eigen::Vector3d endPointLine  = (pose * zDirectionObstacle).head(3);


//         // compare using the two methods
//         REQUIRE( fabs((basePointLine - basePointLink).norm()) < 0.01);
//         REQUIRE( fabs((endPointLine - endPointLink).norm()) < 0.01);
//     }
// }

// TEST_CASE("Kinova_arm set position global move", "[arm]") {
//     Eigen::Matrix4d basePosition;
//     basePosition << 1, 0, 0, 0.1,
//                     0, 1, 0, 0.2,
//                     0, 0, 1, 0.4,
//                     0, 0, 0, 1;
//     KinovaArm kinovaArm(urdf_filename, basePosition);
//     std::vector<double> testPose = {deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(30),
//                                     deg2rad(30), deg2rad(30), deg2rad(30)};
//     kinovaArm.updatePose(testPose);
//     Eigen::Matrix4d link2Pose;
//     Eigen::Matrix4d endLinkPose;
//     link2Pose << 1,       0, 0.417668, 0.0973125,
//                  0, 1.15523,-0.276125,  0.195344,
//                  0,       0, 0.865626,   0.68481,
//                  0,       0,        0,         1;

//     endLinkPose <<  1,       0,  0.573497,   0.385814,
//                     0, 1.87086, -0.620803, -0.0515065,
//                     0,       0,  0.534514,    1.16358,
//                     0,       0,         0,          1;

//     REQUIRE( fabs((link2Pose - kinovaArm.links[2]->pose).norm()) < 0.1);
//     REQUIRE( fabs((endLinkPose - kinovaArm.links[5]->pose).norm()) < 0.1);
// }

// TEST_CASE("Kinova_arm test link positions global move", "[arm]") {
//     Eigen::Matrix4d basePosition;
//     basePosition << 1, 0, 0, 0.1,
//                     0, 1, 0, 0.2,
//                     0, 0, 1, 0.4,
//                     0, 0, 0, 1;
//     KinovaArm kinovaArm(urdf_filename, basePosition);
//     std::vector<double> testPose = {deg2rad(90), deg2rad(90), deg2rad(90), deg2rad(90),
//                                     deg2rad(90), deg2rad(90), deg2rad(90)};
//     kinovaArm.updatePose(testPose);

//     Eigen::Vector4d origin(0, 0, 0, 1);

//     for(int i=0; i < kinovaArm.nJoints-1; i++) {
//         // get endpoints from the frames calculation
//         Eigen::Matrix4d baseMatLink = kinovaArm.frameToMatrix(*kinovaArm.localPoses[i]);
//         Eigen::Matrix4d endMatLink = kinovaArm.frameToMatrix(*kinovaArm.localPoses[i+1]);
//         Eigen::Vector3d basePointLink = (baseMatLink * origin).head(3);
//         Eigen::Vector3d endPointLink = (endMatLink * origin).head(3);

//         // Get the pose from these points using the same method in kinovaArm
//         Eigen::Matrix4d pose = kinovaArm.linkFramesToPose(*kinovaArm.localPoses[i], *kinovaArm.localPoses[i+1]);

//         // get endpoints from the pose calculation
//         Eigen::Vector4d zDirectionObstacle(0, 0, kinovaArm.lengths[i], 1);
//         Eigen::Vector3d basePointLine = (pose * origin).head(3);
//         Eigen::Vector3d endPointLine  = (pose * zDirectionObstacle).head(3);

//         // compare using the two methods
//         REQUIRE( fabs((basePointLine - basePointLink).norm()) < 0.1);
//         REQUIRE( fabs((endPointLine - endPointLink).norm()) < 0.1);
//     }
// }

// TEST_CASE("Kinova_arm test inverse kinematics", "[arm]") {
//     KinovaArm kinovaArm(urdf_filename);
//     std::vector<double> testPose = {deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(30),
//                                     deg2rad(30), deg2rad(30), deg2rad(30)};
//     std::vector<double> outputPose = {0.768989, 2.89724, -0.913496, -5.39591, -1.04212,
//                                       2.87854, 0.806753};
//     kinovaArm.updatePose(testPose);

//     double x, y, z, alpha, beta, gamma;

//     x = 1;
//     y = 0;
//     z = 0;
//     alpha = 0;
//     beta = 0;
//     gamma = 0;


//     std::vector<double> output;
//     KDL::Vector pos(x, y, z);
//     KDL::Vector rot(alpha, beta, gamma);
//     KDL::Twist twist(pos, rot);
//     double difference;

//     output = kinovaArm.ikVelocitySolver(twist);
//     for (int i=0; i<output.size(); i++) {
//         // std::cout << "i: " << output[i] << ", " << outputPose[i] << std::endl;
//         difference += fabs(output[i]- outputPose[i]);
//     }
//     REQUIRE( difference < 0.001);
// }

// TEST_CASE("Kinova_arm distance to obstacle", "[monitor]") {

//     KinovaArm kinovaArm(urdf_filename);
//     std::vector<double> testPose = {deg2rad(30), deg2rad(30), deg2rad(30), deg2rad(30),
//                                     deg2rad(30), deg2rad(30), deg2rad(30)};
//     kinovaArm.updatePose(testPose);

//     double radius_1 = 10;
//     double length_1 = 70.710678118654755;

//     Eigen::Matrix4d pose_1;
//     pose_1 << 0.89455844, -0.14058875,  0.42426407,  0,
//                 -0.14058875,  0.81254834,  0.56568542,  0,
//                 -0.42426407, -0.56568542,  0.70710678,  0,
//                 0,          0,          0,          1;

//     Capsule Link_1(pose_1, length_1, radius_1);

//     Monitor monitor(&kinovaArm);
//     monitor.addObstacle(&Link_1);

//     std::vector<std::vector<double>> ToObstacles = monitor.distanceToObjects();
//     std::vector<std::vector<double>> ToLinks = monitor.distanceBetweenArmLinks();


//     std::vector<std::vector<double>>  ObstaclesResult{
//         {-10.04, -9.92939, -9.83483, -9.69385, -9.53986, -9.36271, -9.26948, -9.17337 }
//     };
//     std::vector<std::vector<double>>  LinksResult{
//         {0,-0.08,0.0484925,0.248115,0.456469,0.636643,0.73324,0.805921},
//         {-0.08,0,-0.0798875,0.130574,0.341038,0.531145,0.631666,0.710244},
//         {0.0484925,-0.0798875,0,-0.0799034,0.130573,0.324834,0.427848,0.512702},
//         {0.248115,0.130574,-0.0799034,0,-0.0799034,0.128611,0.234511,0.330082},
//         {0.456469,0.341038,0.130573,-0.0799034,0,-0.0799025,0.0260276,0.124736},
//         {0.636643,0.531145,0.324834,0.128611,-0.0799025,0,-0.0799999,0.0259303},
//         {0.73324,0.631666,0.427848,0.234511,0.0260276,-0.0799999,0,-0.0799999},
//         {0.805921,0.710244,0.512702,0.330082,0.124736,0.0259303,-0.0799999,0}
//     };

//     for(int i=0; i < ToObstacles.size(); i++ ) {
//         double norm = 0;
//         for(int j=0; j< ToObstacles[i].size(); j++){
//             norm += std::pow(fabs(ToObstacles[i][j]-ObstaclesResult[i][j]), 2);
//         }
//         REQUIRE(norm < 0.1);
//     }
            
//     for(int i=0; i < ToLinks.size(); i++ ) {
//         double norm = 0;
//         for(int j=0; j< ToLinks[i].size(); j++){
//             norm += std::pow(fabs(ToLinks[i][j]-LinksResult[i][j]), 2);
//         }
//         REQUIRE(norm < 0.1);
//     }
// }


TEST_CASE("Base distance test 1", "[Base - Sphere ]" ) {
    double radius_1 = 12;
    Eigen::Matrix4d pose_1;
    pose_1 << -0.21308182,   0.24106113,  -0.94682927,  10,
                0.24106113,   0.95209683,   0.18815197, 20,
                0.94682927,  -0.18815197,  -0.26098499,  0,
                0,           0,           0,           1;     
     
    Eigen::Vector3d center; //(0.0,0.0,0.0);
     center[0]=0.0;
     center[1]=0.0;
     center[2]=0.0;

    Eigen::Vector3d min={0.66,0.60,0.30}; //(0.0,0.0,0.0);
    Eigen::Vector3d max={}; //(0.0,0.0,0.0); 
    double x=0.33;
    double y=0.3;
    double z=0.15;
    Sphere *Sphere_1=new Sphere(pose_1, radius_1);
    Box3 *mybox = new Box3(center, x, y,z);

    //std::cout << Sphere_1->getShortestDistance(Link_2) << std::endl;
    Eigen::Vector3d Corners[8];

     std::cout<<"my point min point"<<std::endl<<mybox->minPoint<<std::endl;
        std::cout<<"my point max point"<<std::endl<<mybox->maxPoint<<std::endl;
      for (int i = 0; i < 8; i++)
    {
        
           Corners[i]=mybox->CornerPoint(i); 
             
        std::cout<< "Corner point " <<i<<" = "<<std::endl<<Corners[i]<<std::endl;
        
       
    }
    //     // std::cout<<"my point"<< Eigen::Vector3d(mybox->minPoint[0], mybox->minPoint[1],mybox-> maxPoint[2])<<std::endl;
	// 	// case 2: return Eigen::Vector3d(minPoint[0], maxPoint[1], minPoint[2]);
	// 	// case 3: return Eigen::Vector3d(minPoint[0], maxPoint[1], maxPoint[2]);
	// 	// case 4: return Eigen::Vector3d(maxPoint[0], minPoint[1], minPoint[2]);
	// 	// case 5: return Eigen::Vector3d(maxPoint[0], minPoint[1], maxPoint[2]);
	// 	// case 6: return Eigen::Vector3d(maxPoint[0], maxPoint[1], minPoint[2]);
          
    // }
    std::cout << mybox->getShortestDistance(Sphere_1) << std::endl;

    // REQUIRE( Sphere_1->getShortestDistance(Link_2) == Approx(5.9).margin(0.1) );
    REQUIRE( mybox->getShortestDistance(Sphere_1) == Approx(10.1).margin(0.1));

    delete mybox;
    delete Sphere_1;
}



TEST_CASE("Base distance test 2", "[Base - Sphere ]" ) {
    double radius_1 = 12;
    Eigen::Matrix4d pose_1;
    pose_1 << -0.21308182,   0.24106113,  -0.94682927,  10,
                0.24106113,   0.95209683,   0.18815197, 20,
                0.94682927,  -0.18815197,  -0.26098499,  0,
                0,           0,           0,           1;     
     
    Eigen::Vector3d center; //(0.0,0.0,0.0);
     center[0]=0.0;
     center[1]=0.0;
     center[2]=0.0;
    Sphere *Sphere_1=new Sphere(pose_1, radius_1);
    Box3 *mybox = new Box3(center);

    // std::cout << Sphere_1->getShortestDistance(Link_2) << std::endl;
    std::cout << mybox->getShortestDistance(Sphere_1) << std::endl;

    // REQUIRE( Sphere_1->getShortestDistance(Link_2) == Approx(5.9).margin(0.1) );
    REQUIRE( mybox->getShortestDistance(Sphere_1) == Approx(10.3).margin(0.1));

    delete mybox;
    delete Sphere_1;
}



TEST_CASE("Base distance test 3", "[Base - Sphere ]" ) {
    double radius_1 = 12;
    Eigen::Matrix4d pose_1;
    pose_1 << -0.21308182,   0.24106113,  -0.94682927,  3,
                0.24106113,   0.95209683,   0.18815197, 3,
                0.94682927,  -0.18815197,  -0.26098499,  0,
                0,           0,           0,           1;     
     
    Eigen::Vector3d center; //(0.0,0.0,0.0);
     center[0]=0.0;
     center[1]=0.0;
     center[2]=0.0;
    Sphere *Sphere_1=new Sphere(pose_1, radius_1);
    Box3 *mybox = new Box3(center);

    //std::cout << Sphere_1->getShortestDistance(Link_2) << std::endl;
    std::cout << mybox->getShortestDistance(Sphere_1) << std::endl;

    // REQUIRE( Sphere_1->getShortestDistance(Link_2) == Approx(5.9).margin(0.1) );
    REQUIRE( mybox->getShortestDistance(Sphere_1) == Approx(10.3).margin(0.1));

    delete mybox;
    delete Sphere_1;
}





TEST_CASE("Base distance test 4", "[Base - Capsule ]" ) {
    double radius_1 = 12;
    Eigen::Matrix4d pose_1;
    pose_1 << -0.21308182,   0.24106113,  -0.94682927,  80,
                0.24106113,   0.95209683,   0.18815197, 15,
                0.94682927,  -0.18815197,  -0.26098499,  0,
                0,           0,           0,           1;     
     
    Eigen::Vector3d center; //(0.0,0.0,0.0);
     center[0]=0.0;
     center[1]=0.0;
     center[2]=0.0;
     double length_1 = 5;
    Primitive *capsule_1 = new Capsule(pose_1, length_1, radius_1);
    
    Box3 *mybox = new Box3(center);

    REQUIRE( capsule_1->getShortestDistance(mybox) == Approx(4.3).margin(0.1) );
    REQUIRE( mybox->getShortestDistance(capsule_1) == Approx(4.3).margin(0.1));

    delete mybox;
    delete capsule_1;
}