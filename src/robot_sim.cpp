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
        int32_t curTime;
        int32_t prevTime;
        int32_t deltaT;
        std::vector<double> jointVelocities;
        sensor_msgs::JointState jointStates;
        

        KinovaSimulator(){

            ros::NodeHandle np;

            np.param<std::string>("/velocity_topic", inputTopic, "joint_command");
            np.param<std::string>("/joint_state_topic", outputTopic, "joint_state");
        
            curTime = ros::Time::now().toNSec();
            prevTime = curTime;
            for (int i=0; i<6; i++) {
                jointStates.velocity.push_back(0.0);
                jointStates.position.push_back(0.0);
            }
            std::cout << "inputTopic: " << inputTopic << std::endl;

            ros::NodeHandle n;

            subscriber = n.subscribe(inputTopic, 1000, &KinovaSimulator::subVelocity, this);
            velPub = n.advertise<sensor_msgs::JointState>(outputTopic, 1000);
        }

        ~KinovaSimulator() {}

        void subVelocity(const sensor_msgs::JointState::ConstPtr& msg){
            jointVelocities.assign(msg->position.begin(), msg->position.end());
            curTime = ros::Time::now().toNSec();
            if (curTime > prevTime) {
                deltaT = curTime -prevTime;
            }
            else {
                deltaT = 0;
            }
            
            for (int i=0; i<jointVelocities.size(); i++) {
                jointStates.position[i] += jointVelocities[i] * (double)deltaT / 1000;
                jointStates.velocity[i] = jointVelocities[i];
            }
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