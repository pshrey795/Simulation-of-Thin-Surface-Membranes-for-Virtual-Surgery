#include "../include/particle.hpp"

Particle::Particle(vec3 pos, double mass){
    this->position = pos;
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3();
    this->netForce = vec3();
    this->isFixed = false;
    this->edge = NULL;
}

Particle::Particle(vec3 pos, Edge* e, double mass){
    this->position = pos;
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3();
    this->netForce = vec3();
    this->isFixed = false;
    this->edge = e;
}

void Particle::fixParticle(){
    this->isFixed = true;
}

vec3 Particle::calculateExternalForce(){
    return (this->m * GRAVITY);
}

vec3 Particle::calculateInternalForce(){
    vec3 force = vec3();
    
    return force;
}

void Particle::calculateForce(){
    this->netForce = this->calculateExternalForce() + this->calculateInternalForce();
}

void Particle::update(double dt){
    this->calculateForce();
    if(!this->isFixed){
        //Forward Euler(unstable)
        this->velocity += this->netForce * this->invM * dt;
        this->position += this->velocity * dt;
    }
}