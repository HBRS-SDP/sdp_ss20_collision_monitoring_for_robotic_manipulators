# ROS
The package that implements the Kinova Arm package in a ros package for
demonstration purposes.

## Documentation
For UML diagrams have a look in the /docs folder and for basic ROS commands see
http://wiki.ros.org/ROS/Tutorials.

## Requirements:
- KDL (https://www.orocos.org/kdl/installation-manual)
- Eigen (http://eigen.tuxfamily.org)
- ROS kinetic (http://wiki.ros.org/)
- CMAKE V3.5

## Packaged dependencies
The kinova kortex library is packaged with this catkin workspace for easy building
and implementation. This is pulled directly from the [ros kortex](https://github.com/kinovarobotics/ros_kortex) 
github page and the associated licences and readme can be found in the catkin/src
directory.


## Building
In the catkin_workspace folder of the repository make sure you have ros 
setup.bash sourced in the current terminal:
```
...\catkin_workspace$ source /opt/ros/kinetic/setup.bash 
```
Then run catkin build (or catkin_make:
```
...\catkin_workspace$ catkin build
```
Finally source the new setup.bash, or add it to your .bashrc to be run on the
terminals startup:
```
...\catkin_workspace$ source devel/setup.bash
```

## Running the software
There are 2 modes the software can be run in, single or dual arm, each with its
separate interface node types.

### Single arm
First make sure that the correct setup.bash file is sourced.
```
...\catkin_workspace$ source devel/setup.bash
```
Then launch the single arm simulation with:
```
...$ roslaunch kinova_arm single_manipulator.launch 
```
This will bring up the rviz display where the kinova arm model can be seen. To
interface with the model the single interfacer node needs to be opened, so in a 
new terminal source the setup file (if you didn't add it to your .bashrc) and 
then run:
```
...$ rosrun kinova_arm KinovaInterface 
```
Where using basic keyboard commands you can set goals and add obstacles to the
environment.

### Dual arm
First make sure that the correct setup.bash file is sourced.
```
...\catkin_workspace$ source devel/setup.bash
```
Then launch the dual arm simulation with:
```
...$ roslaunch kinova_arm dual_manipulators.launch  
```
This will bring up the rviz display where the kinova arm models can be seen. To
interface with the model the dual interfacer node needs to be opened, so in a 
new terminal source the setup file (if you didn't add it to your .bashrc) and 
then run:
```
...$ rosrun kinova_arm DualArmKinovaInterface 
```
Where using basic keyboard commands you can set goals and add obstacles to the
environment.

## UML diagram

![UML Diagram](https://github.com/broccan/sdp_ss20_collision_monitoring_for_robotic_manipulators/blob/master/catkin_workspace/docs/Kinova_Controller_UML.png)
\image html Kinova_Controller_UML.png

## ROS Node diagrams

### Single arm
![Single Arm](https://github.com/broccan/sdp_ss20_collision_monitoring_for_robotic_manipulators/blob/master/catkin_workspace/docs/rosgraph_single.png)
\image html rosgraph_single.png

### Dual arm
![Dual Arm](https://github.com/broccan/sdp_ss20_collision_monitoring_for_robotic_manipulators/blob/master/catkin_workspace/docs/rosgraph_dual.png)
\image html rosgraph_dual.png