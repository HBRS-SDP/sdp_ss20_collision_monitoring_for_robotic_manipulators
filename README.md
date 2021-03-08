# sdp_ws20_collision_monitoring_for_mobile_manipulator
## Members:
- Urvashi Negi
- Zain Ul Haq
- Sreenivasa Hikkal Venugopala

## Client
- Djordje Vukcevic

## Description
This repository is a group submission for MAS Software Development Project. The goal of this project is to extend the existing implementation to perform the collision monitoring with respect to the mobile base. In this work, we extended the existing implementation such that the robot arms do not collide with the body and also implemented a base controller that controls and helps in collision avoidance and monitoring for the robot base.

## Structure
This repository is structured to contain multiple implementations/libraries and
documents for the final submission. The repository has a hierarchial structure
based around the Collision Monitoring library leading to the final ROS package
used for demonstration. The following links list the core functionalities of
this repository.

### Contents
- [Collision Monitoring Library](collision_monitoring/README.md):
    The base library designed to be highly portable and used for any manipulator type
- [Kinova Arm Package](src/README.md):
    The test implementation of the Collision monitoring library designed for a
    Kinova gen3 7DOF manipulator.
- [Collision Avoidance ROS Package](catkin_workspace/README.md):
    The implementation of the obstacle avoidance and tracing algorithm that the
    base library is designed to suit.
- [Narkin Base Collision Monitoring](catkin_workspace/src/narko_kinova_base_collision/README.md):
    The ROS package for visualization and demonstration of extended implementations.
- [Setup Instructions](setup_instructions.pdf): Provides setup instructions for installation of dependency packages and compiling library.
- Documentation & other related works: 
    The last section found in the /deliverables directory contains the paper,
    presentation and API documentation which is submitted along with this code.



### Folder layout
Here is the basic layout of the folder
structure (note: symbolic links are used to for easy modification and tracking):

```
Repository
│
└─── build/                                         Where the KinovaArm package is to be built
|
└─── catkin_workspace/                              The ros catkin workspace (run catkin_make here)
|   |
|   └─── docs/                                      Contains UML diagrams associated with the ROS pkg
|   └─── src/narko_kinova_base_collision/           The ROS package used for testing and demonstration
|
└─── collision_monitoring/                          The base library built for the project
|   |
|   └─── docs/                                      Contains associated UML diagrams
|   └─── include/                                   The library header files
|   └─── src/                                       The libraries source files
|
└─── deliverables/                                  Contains other non-code submittable documents
|   |
|   └─── doxygen/                                   Contains the html API docs and files to make them
|   └─── latex/                                     Contains the latex files for the research paper
|   └─── presentation/                              Contains the final presentation
|
└─── docs/                                          Contains the UML files for the kinova_arm package
└─── include/                                       Contains the header files for kinova_arm and ROS
└─── src/                                           Contains the source files for kinova_arm and ROS
└─── test/                                          Contains the test source files for kinova_arm
└─── urdf/                                          Contains the URDF file used for kinematic calcs
```

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
* We would like to thank Djordje Vukcevic for his support and motivation throughout the course of this project.

## References
This library is an implementation of the following papers:

* [1] Real-Time Obstacle Avoidance for Manipulators and Mobile Robots, Oussama Khatib, 1986.
* [2] Biologically-inspired dynamical systems for movement generation: automatic real-time goal adaptation and obstacle avoidance, H. Hoffmann et al., 2019.
* [3] A fast procedure for computing the distance between complex objects in three-dimensional space, E. G. Gilbert et al., 1988.
* [4] AABB 3DCollisions - https://gdbooks.gitbooks.io/3dcollisions/content/Chapter1/aabb.html, Accessed on 03/07/2021.
