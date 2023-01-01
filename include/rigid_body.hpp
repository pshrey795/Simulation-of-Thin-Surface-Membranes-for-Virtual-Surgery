#ifndef RIGID_BODY_HPP
#define RIGID_BODY_HPP

#include "common.hpp"
#include "draw.hpp"
#include "lighting.hpp"
#include "material.hpp"
#include "shape.hpp"

class RigidBody{
    private:
        //Animation Parameters
        bool isPlaying = false;     
        bool activatePhysics = false;
        int count = 0;                  //Frame counter to control speed of animation 
        double t = 0;                   //Time counter 

        //Debugging
        bool debug;
        bool checkSanity();

        //Mesh Parameters
        vector<vec3> vertices;
        vector<unsigned int> indices;

        //BVH

    public:
        RigidBody(); 
        RigidBody(vector<vec3> Vertices, vector<unsigned int> Indices);

        void update(float dt);
        void processInput(Window &window);

        //For rendering the mesh 
        void renderMesh(); 

        //Debugging
        void printMeshInfo();

        //Material
        Material mat;

        vec3 velocity;
};

#endif