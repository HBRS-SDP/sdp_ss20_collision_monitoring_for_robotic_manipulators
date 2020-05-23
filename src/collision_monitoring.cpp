// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>

// This file is generated by CMAKE for global config variables
#include "collision_monitoring_config.h"

int main(int argc, char* argv[])
{
  if (argc < 2) {

    // report version
    std::cout << argv[0] << " Version " << Collision_Monitoring_VERSION_MAJOR << "."
              << Collision_Monitoring_VERSION_MINOR << std::endl;
    std::cout << "Usage: " << argv[0] << " number" << std::endl;
    return 1;
  }

  // convert input to double
  const double inputValue = atof(argv[1]);

  // calculate square root
  const double outputValue = sqrt(inputValue);
  std::cout << "The square root of " << inputValue << " is " << outputValue
            << std::endl;
  return 0;
}