// Pulled from https://github.com/catchorg/Catch2/blob/master/docs/tutorial.md

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"
#include <vector>
#include <Eigen/Core>
#include <math.h>


#define private public
#include "kinova_arm.h"
#include "primitives.h"
#include "obstacle.h"
#include "monitor.h"

double deg2rad(double v) {
    return v / 180 * M_PI;
}

TEST_CASE("Monitor test", "[obstacle]") {
    double radius_1 = 10;
    double length_1 = 70.710678118654755;
    Eigen::Matrix4d pose_1;
    pose_1 << 0.89455844, -0.14058875,  0.42426407,  0,
                -0.14058875,  0.81254834,  0.56568542,  0,
                -0.42426407, -0.56568542,  0.70710678,  0,
                0,          0,          0,          1;

    Eigen::Matrix4d pose_2;
    pose_2 << 0.76552284, -0.18758173, -0.61545745,  50,
                -0.18758173,  0.84993462, -0.49236596,  0,
                0.61545745,  0.49236596,  0.61545745,  0,
                0,          0,          0,          1;

    // Cylinder Link_1 = new Cylinder(pose_1, length_1, radius_1);
    // std::vector primitives = new std::vector<Obstacle>;
    // primitives->insert(Link_1);
    // Obstacle obstacle = new Obstacle(pose_1, primitives);
    // REQUIRE(obstacle->pose == pose_1);
    // obstacle->updatePose(pose_2);
    // REQUIRE(obstacle->pose == pose_2);
}

std::string urdf_filename = "/home/brennan/SDP/sdp_ss20_collision_monitoring_for_robotic_manipulators/urdf/GEN3_URDF_V12.urdf";

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

        std::cout << "pose:\n"<< pose<<"\nbaseLine:\n"<<basePointLine
                  << "\nendPointLine:\n"<< endPointLine<<"\nbasePointLink:\n"<<basePointLink
                  << "\nendPointLink:\n"<< endPointLink<<std::endl;

        // compare using the two methods
        REQUIRE( fabs((basePointLine - basePointLink).norm()) < 0.1);
        REQUIRE( fabs((endPointLine - endPointLink).norm()) < 0.1);
    }
}