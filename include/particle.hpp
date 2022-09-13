#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include "common.hpp"
#define DEFAULT_MASS 1.0f
#define DEFAULT_REST_LENGTH 0.5f
#define DEFAULT_STIFFNESS 10.0f
#define DEFAULT_DAMPING 5.0f
#define GRAVITY vec3(0.0f, 0.0f, -9.8f)

using namespace std;

struct Spring {
    double restLength;
    double ks;              //Stiffness constant
    double kd;              //Damping constant
    Spring(){
        restLength = DEFAULT_REST_LENGTH;
        ks = DEFAULT_STIFFNESS;
        kd = DEFAULT_DAMPING;
    }
    Spring(double l0, double ks, double kd){
        this->restLength = l0;
        this->ks = ks;
        this->kd = kd;
    }
    Spring(double l0){
        this->restLength = l0;
        this->ks = DEFAULT_STIFFNESS;
        this->kd = DEFAULT_DAMPING;
    }
};

struct Edge{
    struct Particle *startParticle;
    struct Edge *next;
    struct Edge *prev;
    struct Edge *twin;
    struct Face *face;
    struct Spring spring;
    Edge(Particle *startParticle, Face *face, double l0){
        this->startParticle = startParticle;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = face;
        this->spring = Spring(l0);
    }
    Edge(Particle *startParticle, Face *face){
        this->startParticle = startParticle;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = face;
        this->spring = Spring();
    }
    Edge(){
        this->startParticle = NULL;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = NULL;
        this->spring = Spring();
    }
};

//Face stores;
//1) A half edge that is part of the face
//2) A normal
//3) Indices of its vertices
struct Face{
    struct Edge *edge;
    int indices[3];
    bool reMeshed = false;
    Face(int a, int b, int c, bool isRemeshed = true){
        indices[0] = a;
        indices[1] = b;
        indices[2] = c;
        reMeshed = isRemeshed;
        edge = NULL;
    }
    void setFace(int a, int b, int c){
        indices[0] = a;
        indices[1] = b;
        indices[2] = c;
        reMeshed = true;
    }
    Face(){
        edge = NULL;
    }
};

class Particle {

    private:
        double m;               //Mass of the particle
        double invM;            //Inverse of mass of the particle to avoid repetitive divisions 
        vec3 netForce;
        vec3 velocity;          

        //Updating force 
        vec3 calculateExternalForce();
        vec3 calculateInternalForce();
        vec3 calculateSpringForce(Spring s, Particle* p0, Particle* p1);
        void calculateForce();

    public:
        vec3 position;
        Edge* edge;
        bool isFixed;           //Flag to check if the particle is fixed or not
        Particle(vec3 pos, double mass = DEFAULT_MASS);
        Particle(vec3 pos, Edge* e, double mass = DEFAULT_MASS);

        //Updating the particle
        void update(double dt);

        //For intersection purposes 
        double offset = 0.0f;

};

#endif