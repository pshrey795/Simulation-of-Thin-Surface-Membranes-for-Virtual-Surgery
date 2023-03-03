#ifndef RIGID_BODY_HPP
#define RIGID_BODY_HPP

#include "common.hpp"
#include "draw.hpp"
#include "debug.hpp"
#include "lighting.hpp"
#include "material.hpp"
#include "shape.hpp"
#include "intersect.hpp"

#include<assimp/Importer.hpp>
#include<assimp/scene.h>
#include<assimp/postprocess.h>

struct Primitive {
    int v1, v2, v3; 
    Primitive(int x, int y, int z) : v1(x), v2(y), v3(z) {}
};

class RigidBody{
    private:
        //Animation Parameters  
        bool activatePhysics = false;
        int count = 0;                  //Frame counter to control speed of animation 
        double t = 0;                   //Time counter 

        //Debugging
        bool debug;
        bool drawRBD;
        bool checkSanity();

    public:
        RigidBody();
        RigidBody(string filePath);

        void update(float dt);
        void processInput(Window &window);

        //For rendering the mesh 
        void renderMesh(); 

        //Debugging
        void printMeshInfo();

        //Mesh Parameters
        vector<vec3> vertices;
        vector<Primitive> faces;

        //Material
        Material mat;

        //Motion and Collision parameters
        vec3 velocity;      
        //Collision Mode:
        //0 : Using BVH
        //1 : Using sphere
        int collisionMode;
        //Response Mode:
        //0: Velocity Constraint
        //1: Penalty Force
        int responseMode;
        //Approximating sphere 
        Sphere sphere;  
};

#endif