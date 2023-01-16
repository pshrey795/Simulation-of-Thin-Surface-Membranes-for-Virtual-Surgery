#ifndef SHAPE_HPP
#define SHAPE_HPP

#include "common.hpp"
#include "gui.hpp"

struct Sphere{
    vec3 centre;
    double radius;
    Sphere();
    Sphere(vec3 centre, double radius);
};

#endif