#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
#include "primitives.h"
#include "ros/ros.h"
#include "arm_controller.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/String.h"

void armCallback_0(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i=0; i<msg->position.size() ; ++i)
    {
        ROS_INFO_STREAM("robot 0: " << msg->name[i] << " " << msg->position[i]);
    }
}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "pose_listener");
    ros::NodeHandle n;
    ros::Subscriber joint_angles = n.subscribe("/my_gen3/joint_states", 1000, &armCallback_0);
    ros::Publisher trajectory = n.advertise<trajectory_msgs::JointTrajectory>("/my_gen3/gen3_joint_trajectory_controller/command", 1000);

    ros::Rate loop_rate(10);

    int count = 0;

    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;


        
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        // chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;

    }

    return 0;
}