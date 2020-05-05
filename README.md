# sdp_ss20_collision_monitoring_for_robotic_manipulators
## Members:
- Alan Gomez
- Samuel Parra
- Brennan Penfold


## Requirements:
- KDL (https://www.orocos.org/kdl/installation-manual)
- Eigen (http://eigen.tuxfamily.org)
- CMAKE V3.5


## Building
In the root folder create a build folder:
``` ...\sdp_ss20_collision_monitoring_for_robotic_manipulators> mkdir build```
CD into the build folder and run cmake for the first time to configure the build environment:
```
    ...\sdp_ss20_collision_monitoring_for_robotic_manipulators$ cd build
    ...\sdp_ss20_collision_monitoring_for_robotic_manipulators\build$ cmake ..
```
If VS Code is being used the CMAKE Tools extension is helpful as it allows the library to be
built by pressing F7.

Otherwise the following commands need to be run from the build folder:
```
    ...\sdp_ss20_collision_monitoring_for_robotic_manipulators\build$ cmake ..
    ...\sdp_ss20_collision_monitoring_for_robotic_manipulators\build$ cmake --build .
```
