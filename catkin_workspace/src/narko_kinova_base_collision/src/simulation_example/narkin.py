#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg

def start_server():
    rospy.init_node('narkin_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(9.0)
    while not rospy.is_shutdown():
        try:
            frame_info = tfBuffer.lookup_transform('world_frame', 'base_footprint', rospy.Time())
            break
        except:
            print('Could not get world frame')
            rate.sleep()
            continue
        
    frame_names = tfBuffer.all_frames_as_string()
    print(frame_names)
    parent_frame= 'world_frame' 
    child_frame = 'current_link'
    translation = [0,0,0]
    rotation    = [0,0,0,0]
    translation[0] = frame_info.transform.translation.x
    translation[1] = frame_info.transform.translation.y
    translation[2] = frame_info.transform.translation.z
    rotation[0] = frame_info.transform.rotation.w
    rotation[1] = frame_info.transform.rotation.z
    rotation[2] = frame_info.transform.rotation.y
    rotation[3] = frame_info.transform.rotation.z
    print(translation)

if __name__ == '__main__':
    start_server()