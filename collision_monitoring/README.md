# Collision monitoring 
This work is an extension of the existing library '[collision monitoring](https://github.com/HBRS-SDP/sdp_ss20_collision_monitoring_for_robotic_manipulators)'.
The extension here is to add the robot base as additional primitive and also to implement the collision monitoring and avoidance for the movement of base.

This library was developed for the software development project course at the Hochschule Bonn-Rhein-Sieg. 

### Prerequisites
Prerequisites and instructions for compiling the library are provided in setup_instructions.pdf

### Installing
For every type of manipulator you would like to monitor you must 
implement the Arm interface. This ensures that the geometric descriptions 
used to monitor the distances suit your manipulator. Please refer to the 
documentation on how to do so. 

After implementing the arm you can build your project and use the monitor class.
To build the prjoect, follow the instructions provided in setup_instructions.pdf

## Running the tests
After building you may run the test located in the build directory by running 
the command:

```
./build/test/tests
```
## Authors
* **Urvashi Negi** - [urvashi07](https://github.com/urvashi07)
* **Zain Ul Haq** - [zaindroid](https://github.com/zaindroid)
* **Sreenivasa Hikkal Venugopala** - [Sreeni1204](https://github.com/Sreeni1204)

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
* We would like to thank Djordje Vukcevic for his support and motivation throughout the course of this project.
