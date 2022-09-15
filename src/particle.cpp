#include "../include/particle.hpp"

Particle::Particle(vec3 pos, double mass){
    this->position = pos;
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3(0.0f,0.0f,0.0f);
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->isFixed = false;
    this->edge = NULL;
}

Particle::Particle(vec3 pos, Edge* e, double mass){
    this->position = pos;
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3(0.0f,0.0f,0.0f);
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->isFixed = false;
    this->edge = e;
}

vec3 Particle::calculateExternalForce(){
    return (this->m * GRAVITY);
}

void Particle::update(double dt){
    if(!this->isFixed){
        //Forward Euler(unstable)
        this->netForce += this->calculateExternalForce();
        this->velocity += this->netForce * this->invM * dt;
        this->position += this->velocity * dt;
    }
}