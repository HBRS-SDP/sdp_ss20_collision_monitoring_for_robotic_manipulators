# Collision monitoring 

A framework independent c++ library for monitoring the distance from any manipulator to obstacles,
including other manipulators in the workspace. It implement concepts from these
publications:

* Real-Time Obstacle Avoidance for Manipulators and Mobile Robots, Oussama Khatib, 1986.
* Biologically-inspired dynamical systems for movement generation: automatic real-time goal adaptation and obstacle avoidance, H. Hoffmann et al., 2019.

This library was develop for the software development project course at the Hochschule Bonn-Rhein-Sieg. 

## Getting Started

Copy this folder to your project repository, then install the dependencies and the library by following the instructions.


### Prerequisites

To use this library you need the following dependencies:

- [KDL](https://www.orocos.org/kdl/installation-manual)
- [Eigen 3.3.7](http://eigen.tuxfamily.org)
- [CMAKE V3.5](https://cmake.org/download/)

### Installing

First, in CMakeList.txt add the subdirectory and include the header files in 
`include_directories` alongside the dependencies:

```CMake
add_subdirectory(collision_monitoring)

include_directories(
    ...
    collision_monitoring/include
    /usr/include/eigen3
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
    ...
)

```

Second, for every type of manipulator you would like to monitor you must implement the Arm interface. This ensure that the geometric descriptions used to monitor the distances suit your manipulator. Please refer to the documentation on how to do so. 

After implementing the arm you can build your project and use the monitor class. If you have not done so, create a build directory. Then change directories to the build directory and build using cmake:

```
mkdir build
cd build
cmake ..
```

## Running the tests

After building you may run the test located in the build directory by running the command:

```
./build/collision_monitoring/test/test
```

## Built With

* [CMAKE V3.5](https://cmake.org/download/) - Compiler
* [Eigen 3.3.7](http://eigen.tuxfamily.org) - Library for linear algebra
* [KDL](https://www.orocos.org/kdl/installation-manual) - Kinematics and dynamics library

## Authors

* **Alan Gomez** - [alanorlando95](https://github.com/alanorlando95)
* **Samuel Parra** - [samuelpg](https://github.com/samuelpg)
* **Brennan Penfold** - [broccan](https://github.com/broccan)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* We would like to thank our coach Djordje Vukcevic who supported us throughout the development of this library.
