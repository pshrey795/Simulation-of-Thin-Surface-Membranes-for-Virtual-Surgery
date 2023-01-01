#include "../include/shape.hpp"

Sphere::Sphere(){
    this->centre = vec3(0.0f, 0.0f, 0.0f);
    this->radius = 1.0f;
    this->vel = vec3(0.0f, 0.0f, 0.0f);
    this->isActive = false;
}

Sphere::Sphere(vec3 centre, double radius, vec3 vel){
    this->centre = centre;
    this->radius = radius;
    this->vel = vel;
    this->isActive = false;
}

void Sphere::update(float dt){
    if(isActive){
        this->centre += this->vel * dt;
    }
}

void Sphere::processInput(Window &window){
    if(window.isKeyPressed(GLFW_KEY_O)){
        this->isActive = true;
    }
}