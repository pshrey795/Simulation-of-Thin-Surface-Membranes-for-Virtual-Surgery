#include "../include/particle.hpp"
//Particle Class implementations 
Particle::Particle(vec3 pos, double mass){
    this->position = pos;
    this->initPos = pos; 
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3(0.0f,0.0f,0.0f);
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->isFixed = false;
    this->edge = NULL;
}

Particle::Particle(vec3 pos, vec3 initPos, double mass){
    this->position = pos;
    this->initPos = initPos; 
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3(0.0f,0.0f,0.0f);
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->isFixed = false;
    this->edge = NULL;
}

Particle::Particle(vec3 pos, vec3 initPos, vec3 initVel, double mass){
    this->position = pos;
    this->initPos = initPos; 
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = initVel;
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->isFixed = false;
    this->edge = NULL;
}

Particle::Particle(vec3 pos, Edge* e, double mass){
    this->position = pos;
    this->initPos = pos;
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
        this->velocity += this->netForce * dt * invM;
        this->position += this->velocity * dt;
    }
}

void Particle::updateInvM(){
    this->invM = 1 / this->m;
}

vector<Edge*> Particle::getEdges(){
    vector<Edge*> res;
    Edge* currentEdge = this->edge;
    res.push_back(currentEdge);
    if(currentEdge->prev != NULL){
        Edge* nextRightEdge = currentEdge->prev->twin;
        while(nextRightEdge != currentEdge){
            res.push_back(nextRightEdge);
            if(nextRightEdge->prev != NULL){
                nextRightEdge = nextRightEdge->prev->twin;
            }else{
                break;
            }
        }
        if(nextRightEdge != currentEdge){
            Edge* nextLeftEdge = currentEdge->twin->next;
            while(nextLeftEdge != NULL){
                res.push_back(nextLeftEdge);
                nextLeftEdge = nextLeftEdge->twin->next;
            }
        }
    }else{
        Edge* nextLeftEdge = currentEdge->twin->next;
        if(nextLeftEdge != NULL){
            while(nextLeftEdge != currentEdge){
                if(nextLeftEdge != NULL){
                    res.push_back(nextLeftEdge);
                    nextLeftEdge = nextLeftEdge->twin->next;
                }else{
                    break;
                }
            }
        }
    }
    return res; 
}

//Spring implementations 
Spring::Spring(){
    ks = DEFAULT_STIFFNESS;
    kd = DEFAULT_DAMPING;
    p1 = NULL;
    p2 = NULL;
}

Spring::Spring(double ks, double kd){
    this->ks = ks;
    this->kd = kd;
    p1 = NULL;
    p2 = NULL;
}

Spring::Spring(Particle* p1, Particle* p2){
    this->ks = DEFAULT_STIFFNESS;
    this->kd = DEFAULT_DAMPING;
    this->p1 = p1;
    this->p2 = p2;
}

Spring::Spring(Particle* p1, Particle* p2, double ks, double kd){
    this->ks = ks;
    this->kd = kd;
    this->p1 = p1;
    this->p2 = p2;
}

void Spring::addForce(){
    vec3 diff = p2->position - p1->position;
    vec3 velDiff = p2->velocity - p1->velocity;
    double length = diff.norm();

    //Spring force
    double restLength = (p1->initPos - p2->initPos).norm();
    double springForce = ks * (((length + DELTA) / (restLength + DELTA)) - 1);

    //Damping force
    double dampForce = kd * ((velDiff.dot(diff) + DELTA) / (restLength * length + DELTA));

    //Net force
    vec3 netForce = (springForce + dampForce) * diff.normalized();

    //Adding the force
    p1->netForce += netForce/2;
    p2->netForce -= netForce/2;
}


//Edge implementations
Edge::Edge(Particle *startParticle, Face *face){
    this->startParticle = startParticle;
    this->next = NULL;
    this->prev = NULL;
    this->twin = NULL;
    this->face = face;
    this->isBoundary = false;
    this->spring = Spring();
}

Edge::Edge(){
    this->startParticle = NULL;
    this->next = NULL;
    this->prev = NULL;
    this->twin = NULL;
    this->face = NULL;
    this->isBoundary = false;
    this->spring = Spring();
}

void Edge::addForce(){
    this->spring.p1 = this->startParticle;
    this->spring.p2 = this->twin->startParticle;
    return this->spring.addForce();
}


//Face implementations 
Face::Face(){
    edge = NULL;
}

void Face::setFace(int a, int b, int c){
    indices[0] = a;
    indices[1] = b;
    indices[2] = c;
    reMeshed = true;
}

Face::Face(int a, int b, int c, bool isRemeshed){
    indices[0] = a;
    indices[1] = b;
    indices[2] = c;
    reMeshed = isRemeshed;
    edge = NULL;
}