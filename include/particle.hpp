#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include "debug.hpp"
#include "intersect.hpp"

using namespace std; 

class Particle;

struct Spring {
    double ks;              //Stiffness constant
    double kd;              //Damping constant
    Particle* p1;
    Particle* p2;
    Spring();
    Spring(double ks, double kd);
    Spring(Particle* p1, Particle* p2);
    Spring(Particle* p1, Particle* p2, double ks, double kd);
    vec3 addForce();
};

struct Edge{
    struct Particle *startParticle;
    struct Edge *next;
    struct Edge *prev;
    struct Edge *twin;
    struct Edge *ghostTwinCw;
    struct Edge *ghostTwinAcw;
    struct Face *face;
    struct Spring spring;
    int listIdx;
    bool isBoundary;
    bool isActive;
    Edge(Particle *startParticle, Face *face);
    Edge();
    vec3 addForce();
};

//Face stores;
//1) A half edge that is part of the face
//2) A normal
//3) Indices of its vertices
struct Face{
    struct Edge *edge;
    int indices[3];
    bool reMeshed = false;
    vec3 color; 
    int listIdx;
    int helperIdx;
    Face(int a, int b, int c);
    void setFace(int a, int b, int c);
    Face();
};

//Stores the details of a velocity constraint
struct Constraint{
    bool isActive;
    vec3 velocity;
    Constraint();
    Constraint(vec3 velocity);
};

enum TimeIntegrationType {
    FWD_EULER,
    BWD_EULER,
};

class Particle {

    private:        

    public:
        unsigned int listIdx;
        double m;               //Mass of the particle
        double invM;            //Inverse mass of the particle
        vec3 position;
        vec3 velocity;
        vec3 netForce; 
        vec3 externalForce;
        Edge* edge;
        Particle(vec3 pos, double mass = DEFAULT_MASS);
        Particle(vec3 pos, vec3 initPos, double mass = DEFAULT_MASS);
        Particle(vec3 pos, vec3 initPos, vec3 initVel, double mass = DEFAULT_MASS);
        Particle(vec3 pos, Edge* e, double mass = DEFAULT_MASS);

        //Getting all outgoing edges from the current particle
        vector<Edge*> getEdges();

        //Storing the initial position of the particle 
        vec3 initPos; 

        //Updating the particle
        vec3 calculateExternalForce();
        void updatePos(double dt);
        void updateInvM();

        //For intersection purposes 
        double offset = 0.0f;

        //Constraint status
        Constraint constraint;

        //Fracture related parameters
        bool crackTip;

};

//Helper functions 
void calculateForce(Spring& s, vecX& f, matX& Jx, matX& Jv);

#endif