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
        this->velocity += this->netForce * this->invM * dt;
        this->position += this->velocity * dt;
    }
}

vector<Particle*> Particle::getNeighbors(){
    vector<Particle*> res;
    Edge* currentEdge = this->edge;
    res.push_back(currentEdge->twin->startParticle);

    //Traversing towards the right(counter clockwise)
    if(currentEdge->prev != NULL){
        Edge* rightEdge = currentEdge->prev->twin;
        while(rightEdge != currentEdge){
            res.push_back(rightEdge->twin->startParticle);
            if(rightEdge->prev != NULL){
                rightEdge = rightEdge->prev->twin;
            }else{
                break;
            }
        }
        if(rightEdge != currentEdge){
            //Traversing towards the left(clockwise)
            Edge* leftEdge = currentEdge->next->twin;
            while(leftEdge != currentEdge){
                if(leftEdge != NULL){
                    res.push_back(leftEdge->twin->startParticle);
                    leftEdge = leftEdge->twin->next;
                }else{
                    break;
                }
            }
        }
    }else{
        //Traversing towards the left(clockwise)
        Edge* leftEdge = currentEdge->next->twin;
        while(leftEdge != currentEdge){
            if(leftEdge != NULL){
                res.push_back(leftEdge->twin->startParticle);
                leftEdge = leftEdge->twin->next;
            }else{
                break;
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
    double springForce = ks * ((length / restLength) - 1);

    //Damping force
    double dampForce = kd * ((velDiff.dot(diff.normalized())) / (restLength));

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
    this->spring = Spring();
}

Edge::Edge(){
    this->startParticle = NULL;
    this->next = NULL;
    this->prev = NULL;
    this->twin = NULL;
    this->face = NULL;
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

//Static helper functions 
vec3 getInitPosAtPoint(Particle* p){
    return p->initPos;
}

vec3 getInitPosAtEdge(Edge* e, vec3 pos){
    vec3 ip1 = e->startParticle->initPos;
    vec3 ip2 = e->twin->startParticle->initPos;
    vec3 p1 = e->startParticle->position;
    vec3 p2 = e->twin->startParticle->position;
    double t = (pos - p1).norm() / (p2 - p1).norm(); 
    return (1 - t) * ip1 + t * ip2;
}