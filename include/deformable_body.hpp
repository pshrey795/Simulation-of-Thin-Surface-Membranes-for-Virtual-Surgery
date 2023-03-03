#ifndef MESH_HPP
#define MESH_HPP

#include<bits/stdc++.h>
#include "draw.hpp"
#include "camera.hpp"
#include "path.hpp"
#include "half_edge.hpp"

using namespace std;
using namespace Eigen;

class DeformableBody{
    private:
        TimeIntegrationType timeIntegrationType = BWD_EULER;

        //Animation Parameters
        bool isPlaying = false;
        bool toCut = false;  
        bool startCut = false;
        bool activatePhysics = false;
        int count = 0;                  //Frame counter to control speed of animation 
        double t = 0;                   //Time counter 

        //Cut Parameters
        Path* currentPath = NULL;
        unsigned int currIntersectIdx;
        vec3 upVec;                     //Vertical vector, to be decided from the instrument
        void setupPath();
        
        //Cut Graph
        //List of nodes, store only the indices of the cut faces
        vector<int> cutList;
        //Cut Graph stores the graph of faces for determining the intersection points
        //In general, an undirected graph, so stored as an adjacency list
        //Currently, we expect a single cycle, so store simply as a list
        // vector<vector<int>> cutGraph;    
        vector<int> cutGraph;
        void constructCutGraph();
        void getIntersectionPts();
        void processCut();

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

    public:
        DeformableBody();

        //Mesh Parameters
        HalfEdge *mesh;

        //Visual Debugging
        int drawMode;
        int debugMode;

        //Simulation and Interaction 
        void update(float dt);
        void processInput(Window &window);

        //For rendering the mesh 
        void renderMesh(); 
        void renderDebugMesh();

        //Debugging
        void printMeshInfo();

        //To check sanity of the half edge data structure representing this mesh
        bool debug;
        bool checkSanity();
};

#endif