// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
#include "primitives.h"
#include "ros/ros.h"
#include <cstdio>
#include <ros/time.h>
#include "time.h"
#include "std_msgs/Float64.h"
#include "arm_controller.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include <math.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include "marker_publisher.h"
std::string goalTopic;
class KinovaSimulator
{
    public:
        std::string inputTopic;
        std::string outputTopic;
        geometry_msgs::Twist speed; 

        Eigen::Vector3d goal;

        float x=0.0;
        float y=0.0;
        float th=0.0;
        ros::Subscriber subscriber;
        ros::Publisher velPub;
        ros::Publisher motion_pub;
        ros::Subscriber goalSub;
        ros::Subscriber sub;
        uint64_t curTime;
        uint64_t prevTime;
        uint64_t deltaT;
        std::vector<double> jointVelocities;
        std::vector<double> jointPositions;
        sensor_msgs::JointState jointStates;
        ros::Time time;
		
		
        void goalCallback(const geometry_msgs::Point::ConstPtr& msg){
            goal[0] = msg->x;
            goal[1] = msg->y;
            goal[2] = msg->z;
            std::cout<<"New goal point found: "<< goal<<std::endl;
        }
        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
            tf2::Quaternion q_orig, q_rot, q_new;
                x=msg->pose.pose.position.x;
                y=msg->pose.pose.position.y;
            tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
            th= yaw; 
            double inc_x= goal[0] - x;
            double  inc_y= goal[1] - y ;
            double  inc_th= goal[2]-th;
            speed.linear.x=inc_x*0.0;
            speed.linear.y=inc_y*0.0;
            speed.angular.z=inc_th*0.0;
        }
        

        KinovaSimulator(){

            ros::NodeHandle np;
            ros::Duration(3).sleep();

            np.param<std::string>(ros::this_node::getNamespace()+"/velocity_topic", inputTopic, ros::this_node::getNamespace()+"/joint_command");
            np.param<std::string>(ros::this_node::getNamespace()+"/joint_state_topic", outputTopic, ros::this_node::getNamespace()+"/joint_states");
        
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
                jointStates.position.push_back(M_PI_2);
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

            time = ros::Time::now();
            curTime = time.toNSec();
            if (curTime > prevTime) {
                deltaT = curTime -prevTime;
            }
            else {
                deltaT = 0;
            }
            std::cout << " current Pose, current velocity, next pose " << ((double)deltaT / 1000000000.0) << std::endl;
            for (int i=0; i<jointVelocities.size(); i++) {
                double prevPosition = jointStates.position[i];
                jointStates.position[i] += jointVelocities[i] * ((double)deltaT / 1000000000.0);
                jointStates.velocity[i] = jointVelocities[i];
                std::cout << prevPosition << ", " << jointVelocities[i] << ", " << jointStates.position[i] << std::endl;
            }
            jointStates.header.stamp.nsec = time.toNSec();
            jointStates.header.stamp.sec = time.toSec();
            velPub.publish(jointStates);
            prevTime = curTime;
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinova_simulator");
    // setup the first monitor function
    ros::NodeHandle n1;

    // Create the armController class based off the first monitor
    KinovaSimulator kinovaSimulator = KinovaSimulator();

    ros::spin();

    return 0;
}