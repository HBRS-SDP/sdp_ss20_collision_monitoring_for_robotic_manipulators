# Kinova arm
The package that implements the Collision monitoring library for the Kinova gen3
7DOF arm. This not only is done for testing but also gives an example of how
the Arm interface can be implemented.

## Documentation
For UML diagrams have a look in the /docs folder. A html API style 
documentation can be found under /deliverables/doxygen/html

## Requirements:
- KDL (https://www.orocos.org/kdl/installation-manual)
- Eigen (http://eigen.tuxfamily.org)
- CMAKE V3.5


## Building
In the root folder of the repository create a build folder:
```
...$ mkdir build
```
cd into the build folder and run cmake for the first time to configure the build environment:
```
    ...$ cd build
    ...\build$ cmake ..
```
If VS Code is being used the CMAKE Tools extension is helpful as it allows the library to be
built by pressing F7.

Otherwise the following commands need to be run from the build folder:
```
    ...\build$ cmake ..
    ...\build$ cmake --build .
```

## Testing
The test files are built in build/tests and need to be run from the build directory
because of a relative file reference. These can be run using the following commands:
```
    ...\build$ ./test/tests
```

## Notes on files:
The only following files are used in kinova_arm and other files present in the 
/src and /include directories are for the ROS implementation. For more 
information on the ROS implementation see the ROS_Package_README.md.

- kinova_arm.cpp
- kinova_arm.h

## UML diagram

![UML Diagram](https://github.com/broccan/sdp_ss20_collision_monitoring_for_robotic_manipulators/blob/master/docs/Kinova_arm_UML.png)

\image html Kinova_arm_UML.png