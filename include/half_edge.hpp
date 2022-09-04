#ifndef HALFEDGE_HPP
#define HALFEDGE_HPP

#include "common.hpp"
#include<bits/stdc++.h>

using namespace std;
using namespace Eigen;

#define EPSILON 0.04
#define DELTA 0.01

struct Edge{
    struct Vertex *startVertex;
    struct Edge *next;
    struct Edge *prev;
    struct Edge *twin;
    struct Face *face;
    Edge(Vertex *startVertex, Face *face){
        this->startVertex = startVertex;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = face;
    }
    Edge(){
        this->startVertex = NULL;
        this->next = NULL;
        this->prev = NULL;
        this->twin = NULL;
        this->face = NULL;
    }
};

//Vertex stores:
//1) 3D coordinate
//2) An outgoing half edge
struct Vertex{
    vec3 position;
    struct Edge *edge;
    double offset;
    Vertex(vec3 pos){
        position = pos;
        edge = NULL;
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

struct Plane{
    vec3 origin;
    vec3 normal;
    Plane(vec3 origin, vec3 normal){
        this->origin = origin;
        this->normal = normal.normalized();
    }
};

class HalfEdge {

    public:
        //Vertices, Edges and Faces
        vector<Vertex*> vertex_list;
        vector<Edge*> edge_list;
        vector<Face*> face_list;
        vector<vector<int>> adjList;
    
        //Constructor 
        HalfEdge();
        HalfEdge(vector<vec3> Vertices, vector<int> Indices);

        //Obtaining the intersection points of the edges
        //Each element is a tuple with the following elements
        //1) vec3 : intersection point
        //2) int : type of the intersection point(explained later in re-meshing)
        //3) int : index of the relevant data structure which stores the locality info
        // of the intersection point
        vector<tuple<vec3, int, int>> Intersect(Plane plane);

        //Re Mesh
        //Intersection Record (point, type, edge of intersection) (Type 0: Vertex, Type 1: Edge, Type 2: Face)
        //Type -1 if last or first point

        //1) Intersection Record of current point
        //2) Intersection Record of last point
        //3) Intersection Record of next point
        //2) Vertex of the last intersection
        //4) Left cross edge of the split of the last intersection point
        //5) Right cross edge of the split of the last intersection point
        //6) Left side edge of the split of the last intersection point
        //7) Right side edge of the split of the last intersection point
        //8) Direction of normal of plane for splitting
        //Cross Edge: Edge between current intersection point and last intersection point
        //Side Edge: Split edges on the edge of the current intersection point
        void reMesh(tuple<vec3, int, int> intPt, tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> nextIntPt, Vertex* &lastVertex, Edge* &leftCrossEdge, Edge* &rightCrossEdge, Edge* &leftSideEdge, Edge* &rightSideEdge, vec3 normal);

    private:
        //Helper functions for checking intersection
        double triArea(vec3 a, vec3 b, vec3 c);
        bool isInside(Face* face, vec3 point);
        void VertexOffsets(Plane plane);
        vector<int> IntersectingEdges();
        vector<tuple<vec3,int,int>> IntersectingVertices(vector<int> edges);

};

#endif