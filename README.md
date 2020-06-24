# sdp_ss20_collision_monitoring_for_robotic_manipulators
## Members:
- Alan Gomez
- Samuel Parra
- Brennan Penfold

## Client
- Djordje

## Description
This repository is a group submission for MAS Software Development Project. The
goal of the project is to implement a given software algorithm from a research
paper, while communicating with a client to add desired features. The paper
implemented is a method of obstacle avoidance for robotic manipulators and 
at the request of the client a combination of 2 papers was implemented and can
be found in the reference section \[1\]\[2\]



## Structure
This repository is structured to contain multiple implementations/libraries and
documents for the final submission. The repository has a hierarchial structure
based around the Collision Monitoring library leading to the final ROS package
used for demonstration. The following links list the core functionalities of
this repository.

### Contents
- Collision Monitoring Library:
- Kinova Arm Package:
- Collision Avoidance ROS Package:
- 


### Folder layout
Here is the basic layout of the folder
structure (note: symbolic links are used to for easy modification and tracking):

/Repository/

    -/build/                    Where the KinovaArm package is to be built

        -/test/tests            The test file based off catch

    -/catkin_workspace/         The ros catkin workspace (run catkin_make here)

        -/docs/                 Contains UML diagrams associated with the ROS pkg
        -/src/kinova_arm/       The ROS package used for testing and demonstration

    -/collision_monitoring/     The base library built for the project

        -/docs/                 Contains associated UML diagrams
        -/include/              The library header files
        -/src/                  The libraries source files

    -/deliverables/             Contains other non-code submittable documents

        -/doxygen/              Contains the html API docs and files to make them
        -/latex/                Contains the latex files for the research paper
        -/presentation/         Contains the final presentation

    -/docs/                     Contains the UML files for the kinova_arm package
    -/include/                  Contains the header files for kinova_arm and ROS
    -/src/                      Contains the source files for kinova_arm and ROS
    -/test/                     Contains the test source files for kinova_arm
    -/urdf/                     Contains the URDF file used for kinematic calcs


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

## References
This library is an implementation of the following papers:

* [1] Real-Time Obstacle Avoidance for Manipulators and Mobile Robots, Oussama Khatib, 1986.
* [2] Biologically-inspired dynamical systems for movement generation: automatic real-time goal adaptation and obstacle avoidance, H. Hoffmann et al., 2019.
