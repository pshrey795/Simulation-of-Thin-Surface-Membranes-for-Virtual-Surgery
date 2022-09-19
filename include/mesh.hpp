#ifndef MESH_HPP
#define MESH_HPP

#include<bits/stdc++.h>
#include "common.hpp"
#include "draw.hpp"
#include "camera.hpp"
#include "gui.hpp"
#include "path.hpp"
#include "half_edge.hpp"
using namespace std;
using namespace Eigen;

struct Material {
    //To be filled depending on the properties to be captured 
};

class Mesh {
    private:

        //Animation Parameters
        bool isPlaying = false;     
        bool activatePhysics = false;
        int count = 0;                  //Frame counter to control speed of animation 
        double t = 0;                   //Time counter 

        //Mesh Parameters
        HalfEdge *mesh;
        Material mat;

        //Path Parameters
        Path* currentPath = NULL;
        int currIntersectIdx;
        vec3 upVec;                     //Vertical vector, to be decided from the instrument
        void setupPath();
        void setupCut();

        //Storing intersection points of the path with the mesh 
        vector<tuple<vec3, int, int>> intersectPts;
        vector<vec3> normals;

        //For processing the intersection points 
        void removeDuplicates(vector<tuple<vec3,int,int>> &vertices);
        void removeFacePts(vector<tuple<vec3,int,int>> &vertices);
        vector<tuple<vec3, int, int>> filterAndSort(vector<tuple<vec3, int, int>> intersections, vec3 startPoint, vec3 endPoint, bool first);
        vector<tuple<vec3, int, int>> dirSort(vector<tuple<vec3, int, int>> intersections, Plane p);

        //Temporary pointers for book-keeping during remeshing
        Particle* vertexLast;
        Edge* crossEdgeLeft;
        Edge* crossEdgeRight;
        Edge* sideEdgeLeft;
        Edge* sideEdgeRight;

    public:
        Mesh();
        Mesh(vector<vec3> Vertices, vector<unsigned int> Indices);
        void update(float dt);
        void processInput(Window &window);
        void renderMesh(); 

};

#endif