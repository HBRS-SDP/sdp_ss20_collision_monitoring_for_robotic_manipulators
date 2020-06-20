// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
#include "primitives.h"
#include "ros/ros.h"
#include "time.h"
#include "std_msgs/Float64.h"
#include "arm_controller.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"

class KinovaSimulator
{
    public:
        std::string inputTopic;
        std::string outputTopic;


        ros::Subscriber subscriber;
        ros::Publisher velPub;
        uint64_t curTime;
        uint64_t prevTime;
        uint64_t deltaT;
        std::vector<double> jointVelocities;
        std::vector<double> jointPositions;
        sensor_msgs::JointState jointStates;
        ros::Time time;
        

        KinovaSimulator(){

            ros::NodeHandle np;

            np.param<std::string>("/velocity_topic", inputTopic, "joint_command");
            np.param<std::string>("/joint_state_topic", outputTopic, "joint_states");
        
            time = ros::Time::now();
            curTime = time.toNSec();
            prevTime = curTime;
            std::vector<std::string> strings = {"Actuator1", "Actuator2",
                                                "Actuator3", "Actuator4",
                                                "Actuator5", "Actuator6",
                                                "Actuator7"};
            for (int i=0; i<7; i++) {
                jointStates.name.push_back(strings[i]);
                jointStates.velocity.push_back(0.0);
                jointStates.position.push_back(0.0);
            }
            jointStates.header.stamp.nsec = time.toNSec();
            jointStates.header.stamp.sec = time.toSec();

            ros::NodeHandle n;

            subscriber = n.subscribe(inputTopic, 1000, &KinovaSimulator::subVelocity, this);
            velPub = n.advertise<sensor_msgs::JointState>(outputTopic, 1000);
        }

        ~KinovaSimulator() {}

        void subVelocity(const sensor_msgs::JointState::ConstPtr& msg){
            jointVelocities.assign(msg->velocity.begin(), msg->velocity.end());
            jointPositions.assign(msg->position.begin(), msg->position.end());

            time = ros::Time::now();
            curTime = time.toNSec();
            if (curTime > prevTime) {
                deltaT = curTime -prevTime;
            }
            else {
                deltaT = 0;
            }
            std::cout << " current Pose, current velocity, next pose " << curTime << std::endl;
            for (int i=0; i<jointVelocities.size(); i++) {
                jointStates.position[i] = jointPositions[i] + jointVelocities[i] * ((double)deltaT) / 1000000000.0;
                jointStates.velocity[i] = jointVelocities[i];
                std::cout << jointPositions[i] << ", " << jointVelocities[i] << ", " << jointStates.position[i] << std::endl;
            }
            jointStates.header.stamp.nsec = time.toNSec();
            jointStates.header.stamp.sec = time.toSec();
            velPub.publish(jointStates);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinova_simulator");
    // setup the first monitor function


    // Create the armController class based off the first monitor
    KinovaSimulator kinovaSimulator = KinovaSimulator();

    ros::spin();

    return 0;
}