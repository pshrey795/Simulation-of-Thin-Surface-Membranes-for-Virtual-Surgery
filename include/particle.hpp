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
    int p0, p1;             //Indices of the particles
    Spring(int p0, int p1){
        this->p0 = p0;
        this->p1 = p1;
        restLength = DEFAULT_REST_LENGTH;
        ks = DEFAULT_STIFFNESS;
        kd = DEFAULT_DAMPING;
    }
    Spring(int p0, int p1, double l0, double ks, double kd){
        this->p0 = p0;
        this->p1 = p1;
        this->restLength = l0;
        this->ks = ks;
        this->kd = kd;
    }
    Spring(int p0, int p1, double l0){
        this->p0 = p0;
        this->p1 = p1;
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
    Edge(Particle *startParticle, Face *face){
        this->startParticle = startParticle;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = face;
    }
    Edge(){
        this->startParticle = NULL;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = NULL;
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

        //Updating force 
        vec3 calculateExternalForce();

    public:
        vec3 position;
        vec3 velocity; 
        vec3 netForce;
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