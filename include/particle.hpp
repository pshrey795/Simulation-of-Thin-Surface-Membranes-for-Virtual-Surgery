#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include "common.hpp"
#define DEFAULT_MASS 1.0f
#define DEFAULT_STIFFNESS 10.0f
#define DEFAULT_DAMPING 5.0f
#define GRAVITY vec3(0.0f, 0.0f, -9.8f)

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
    void addForce();
};

struct Edge{
    struct Particle *startParticle;
    struct Edge *next;
    struct Edge *prev;
    struct Edge *twin;
    struct Face *face;
    struct Spring spring;
    Edge(Particle *startParticle, Face *face);
    Edge();
    void addForce();
};

//Face stores;
//1) A half edge that is part of the face
//2) A normal
//3) Indices of its vertices
struct Face{
    struct Edge *edge;
    int indices[3];
    bool reMeshed = false;
    Face(int a, int b, int c, bool isRemeshed = true);
    void setFace(int a, int b, int c);
    Face();
};

class Particle {

    private:
        double m;               //Mass of the particle
        double invM;            //Inverse of mass of the particle to avoid repetitive divisions          

        //Updating force 
        vec3 calculateExternalForce();

    public:
        vec3 position;
        vec3 velocity; 
        vec3 netForce;
        Edge* edge;
        bool isFixed;           //Flag to check if the particle is fixed or not
        Particle(vec3 pos, double mass = DEFAULT_MASS);
        Particle(vec3 pos, vec3 initPos, double mass = DEFAULT_MASS);
        Particle(vec3 pos, Edge* e, double mass = DEFAULT_MASS);

        vector<Particle*> getNeighbors();

        //Storing the initial position of the particle 
        vec3 initPos; 

        //Updating the particle
        void update(double dt);

        //For intersection purposes 
        double offset = 0.0f;

};

//Static helper functions 
vec3 getInitPosAtPoint(Particle* p);
vec3 getInitPosAtEdge(Edge* e, vec3 pos);
vec3 getInitPosAtFace(Face* f, vec3 pos);

#endif