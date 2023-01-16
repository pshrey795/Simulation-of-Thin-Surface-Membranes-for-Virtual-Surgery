#include "../include/shape.hpp"

Sphere::Sphere(){
    this->centre = vec3(0.0f, 0.0f, 0.0f);
    this->radius = 1.0f;
}

Sphere::Sphere(vec3 centre, double radius){
    this->centre = centre;
    this->radius = radius;
} 