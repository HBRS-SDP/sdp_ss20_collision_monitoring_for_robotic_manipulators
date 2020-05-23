#include "primitives.h"


Cylinder::Cylinder(std::vector<double> pose, double length, double radius){

}

Cylinder::~Cylinder(){

}

double Cylinder::get_shortest_dist(std::vector<double>){

    return 0;
}

std::vector<double> Cylinder::get_closest_point(std::vector<double>){

    std::vector<double> return_var = {0., 0.};
    return return_var;
}


// N ellipsoide funcitons and declaration

N_ellipsoid::N_ellipsoid(std::vector<double> pose){

}

N_ellipsoid::~N_ellipsoid(){

}

double N_ellipsoid::get_shortest_dist(std::vector<double>){

    return 0;
}

std::vector<double> N_ellipsoid::get_closest_point(std::vector<double>){

}
