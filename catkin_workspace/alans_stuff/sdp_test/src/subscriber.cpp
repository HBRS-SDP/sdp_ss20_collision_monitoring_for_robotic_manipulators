#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

void armCallback_0(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i=0; i<msg->position.size() ; ++i)
    {
      ROS_INFO_STREAM("robot 0: " << msg->name[i] << " " << msg->position[i]);
    }
}

void armCallback_1(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i=0; i<msg->position.size() ; ++i)
    {
      ROS_INFO_STREAM("robot 1: " << msg->name[i] << " " << msg->position[i]);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pose_listener");
    ros::NodeHandle n;
    ros::Subscriber sub_0 = n.subscribe("my_gen3_0/joint_states", 1000, armCallback_0);
    ros::Subscriber sub_1 = n.subscribe("my_gen3_1/joint_states", 1000, armCallback_1);
    ros::spin();

    return 0;
}
// %EndTag(FULLTEXT)%