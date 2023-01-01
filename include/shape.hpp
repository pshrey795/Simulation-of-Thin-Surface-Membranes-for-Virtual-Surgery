#ifndef SHAPE_HPP
#define SHAPE_HPP

#include "common.hpp"
#include "gui.hpp"

struct Sphere{
    vec3 centre;
    double radius;
    vec3 vel; 
    bool isActive; 
    Sphere();
    Sphere(vec3 centre, double radius, vec3 vel);
    void update(float dt);
    void processInput(Window &window);
};

#endif