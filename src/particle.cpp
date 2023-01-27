#include "../include/particle.hpp"
//Particle Class implementations 
Particle::Particle(vec3 pos, double mass){
    this->position = pos;
    this->initPos = pos; 
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3(0.0f,0.0f,0.0f);
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->externalForce = vec3(0.0f, 0.0f, 0.0f);
    this->edge = NULL;
}

Particle::Particle(vec3 pos, vec3 initPos, double mass){
    this->position = pos;
    this->initPos = initPos; 
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3(0.0f,0.0f,0.0f);
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->externalForce = vec3(0.0f, 0.0f, 0.0f);
    this->edge = NULL;
}

Particle::Particle(vec3 pos, vec3 initPos, vec3 initVel, double mass){
    this->position = pos;
    this->initPos = initPos; 
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = initVel;
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->externalForce = vec3(0.0f, 0.0f, 0.0f);
    this->edge = NULL;
}

Particle::Particle(vec3 pos, Edge* e, double mass){
    this->position = pos;
    this->initPos = pos;
    this->m = mass;
    this->invM = 1 / mass;
    this->velocity = vec3(0.0f,0.0f,0.0f);
    this->netForce = vec3(0.0f,0.0f,0.0f);
    this->externalForce = vec3(0.0f, 0.0f, 0.0f);
    this->edge = e;
}

vec3 Particle::calculateExternalForce(){
    return (this->externalForce + this->m * vec3(0.0f, 0.0f, -9.8f));
}

void Particle::updatePos(double dt){
    //Update position regardless of whether the particle is constrained or not
    //Since constraints will be applied on velocity and forces not on positions
    //Also, this equation will be followed regardless of method, explicit or implicit
    this->position += this->velocity * dt;
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

vec3 Spring::addForce(){
    vec3 diff = p2->position - p1->position;
    vec3 velDiff = p2->velocity - p1->velocity;
    double length = diff.norm();

    //Spring force
    double restLength = (p1->initPos - p2->initPos).norm();
    double springForce = ks * (length - restLength);

    //Damping force
    double dampForce = kd * ((velDiff.dot(diff) + DELTA) / (length + DELTA));

    //Net force
    vec3 netForce = (springForce + dampForce) * diff.normalized();

    //Adding the force
    p1->netForce += netForce/2;
    p2->netForce -= netForce/2;

    return netForce;
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

vec3 Edge::addForce(){
    this->spring.p1 = this->startParticle;
    this->spring.p2 = this->twin->startParticle;
    return this->spring.addForce();
}


//Face implementations 
Face::Face(){
    edge = NULL;
    color = vec3(1.0f, 0.0f, 0.0f);
    helperIdx = -1;
}

void Face::setFace(int a, int b, int c){
    indices[0] = a;
    indices[1] = b;
    indices[2] = c;
}

Face::Face(int a, int b, int c){
    indices[0] = a;
    indices[1] = b;
    indices[2] = c;
    helperIdx = -1;
    reMeshed = false;
    edge = NULL;
    color = vec3(1.0f, 0.0f, 0.0f);
}

Constraint::Constraint(){
    isActive = false;
}

Constraint::Constraint(vec3 velocity){
    this->velocity = velocity;
    isActive = true;
}

//Calculating forces/Jacobians for implicit integration
void calculateForce(Spring& s, vecX& f, matX& Jx, matX& Jv){
    unsigned int i = s.p1->listIdx;
    unsigned int j = s.p2->listIdx;
    vec3 xij = s.p1->position - s.p2->position;
    vec3 vij = s.p1->velocity - s.p2->velocity;
    double length = xij.norm();

    //Calculating a reference spring force for debugging
    vec3 refForce = s.addForce();

    //Force calculation
    //Spring Force
    double restLength = (s.p1->initPos - s.p2->initPos).norm();
    double springForce = (-1) * s.ks * (length - restLength);
    //Damping Force 
    double dampForce = (-1) * s.kd * ((vij.dot(xij) + DELTA) / (length + DELTA));
    //Net Force
    vec3 netForce = (springForce + dampForce) * xij.normalized();
    //Update force
    f(i) += netForce;

    //Jx calculation
    vec3 xij_cap = xij.normalized();
    mat3 dij = xij_cap * xij_cap.transpose();
    double factor = max(0.0, (1 - (restLength + DELTA) / (length + DELTA)));
    mat3 J = (-1) * s.ks * (factor * (mat3::Identity() - dij) + (dij));
    Jx(i,i) += J;
    Jx(i,j) -= J;

    //Jv calculation 
    J = (-1) * s.kd * dij;
    Jv(i,i) += J;
    Jv(i,j) -= J;
}