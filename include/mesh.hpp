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

class Mesh {
    private:
        bool isPlaying = false;
        int count = 0;
        double t = 0;
        HalfEdge *mesh;
        Path* currentPath = NULL;
        void setupMesh(vector<vec3> Vertices, vector<int> Indices);
        void setupPath();
        int currIntersectIdx;
        vec3 upVec;
        vector<tuple<vec3, int, int>> intersectPts;
        vector<vec3> normals;
        void removeDuplicates(vector<tuple<vec3,int,int>> &vertices);
        vector<tuple<vec3, int, int>> filterAndSort(vector<tuple<vec3, int, int>> intersections, vec3 startPoint, vec3 endPoint, bool first);
        Vertex* vertexLast;
        Edge* crossEdgeLeft;
        Edge* crossEdgeRight;
        Edge* sideEdgeLeft;
        Edge* sideEdgeRight;

    public:
        Mesh();
        bool changeColor = false; 
        void update(float dt);
        void processInput(Window &window);
        void renderMesh(); 

};

#endif