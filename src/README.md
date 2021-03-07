## Documentation
For UML diagrams have a look in the /docs folder. A html API style 
documentation can be found under /deliverables/doxygen/html

## Requirements:
- KDL (https://www.orocos.org/kdl/installation-manual)
- Eigen (http://eigen.tuxfamily.org)
- CMAKE V3.5


## Building
Follow the instructions provided in the setup_instructions.pdf

## Testing
The test files are built in build/tests and need to be run from the build directory
because of a relative file reference. These can be run using the following commands:
```
    ...\build$ ./test/tests
```