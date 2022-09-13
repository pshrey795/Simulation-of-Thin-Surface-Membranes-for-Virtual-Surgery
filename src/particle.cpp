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

vec3 Particle::calculateSpringForce(Spring s, Particle* p0, Particle* p1){
    vec3 diff = p1->position - p0->position;
    vec3 velDiff = p1->velocity - p0->velocity;
    double length = diff.norm();

    //Spring force
    double springForce = s.ks * ((length / s.restLength) - 1);

    //Damping force
    double dampForce = s.kd * ((velDiff.dot(diff.normalized())) / (s.restLength));

    return (springForce + dampForce) * diff.normalized();
}

vec3 Particle::calculateInternalForce(){
    vec3 force = vec3(0.0f,0.0f,0.0f);
    Edge* currentEdge = this->edge;

    //Adding the force of the current edge 
    force += calculateSpringForce(currentEdge->spring, this, currentEdge->twin->startParticle);

    //Traversing to the right of the current edge
    Edge* nextRightEdge = NULL;
    if(currentEdge->prev != NULL){
        nextRightEdge = currentEdge->prev->twin;
        while(nextRightEdge->prev != NULL && nextRightEdge != currentEdge){
            force += calculateSpringForce(nextRightEdge->spring, this, nextRightEdge->twin->startParticle);
            nextRightEdge = nextRightEdge->prev->twin;
        }
    }

    if(nextRightEdge == NULL){
        if(currentEdge->next != NULL){
            //Traversing to the left of the current edge 
            Edge* nextLeftEdge = currentEdge->twin->next;
            while(nextLeftEdge->next != NULL){
                force += calculateSpringForce(nextLeftEdge->spring, this, nextLeftEdge->twin->startParticle);
                nextLeftEdge = nextLeftEdge->twin->next;
            }
        }
    }

    return force;
}

void Particle::calculateForce(){
    this->netForce = this->calculateExternalForce() + this->calculateInternalForce();
}

void Particle::update(double dt){
    if(!this->isFixed){
        //Forward Euler(unstable)
        this->calculateForce();
        this->velocity += this->netForce * this->invM * dt;
        this->position += this->velocity * dt;
    }
}