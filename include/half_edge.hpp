#ifndef HALFEDGE_HPP
#define HALFEDGE_HPP

#include "particle.hpp"

using namespace std;
using namespace Eigen;

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
        vector<Particle*> particle_list;
        vector<Edge*> edge_list;
        vector<Face*> face_list;
        vector<vector<int>> adjList;
    
        //Constraints
        unordered_set<int> constraints;

        //Dynamic information of the particles
        vector<Spring> ghostSprings;

        //Constructor 
        HalfEdge();
        HalfEdge(vector<vec3> Vertices, vector<unsigned int> Indices);
        HalfEdge(vector<vec3> Vertices, vector<unsigned int> Indices, unordered_map<int, vec3> Constraints);

        //Obtaining the intersection points of the edges
        //Each element is a tuple with the following elements
        //1) vec3 : intersection point
        //2) int : type of the intersection point(explained later in re-meshing)
        //3) int : index of the relevant data structure which stores the locality info
        // of the intersection point
        vector<tuple<vec3, int, int>> Intersect(Plane plane);
        vector<tuple<vec3, int, int>> updateIntersectionPts(vector<tuple<vec3, int, int>> intersections);
        
        //Alternate cutting algorithm 
        //Intersection Record (point, type, vertex/edge of intersection) (Type 0: Vertex, Type 1: Edge, Type 2: Face)
        //Type -1 if last or first point

        //1) Intersection Record of the previous point(just to check if this is the first vertex)
        //2) Intersection Record of the current point 
        //3) Intersection Record of the next point
        //4) Direction of normal of the plane of splitting
        void reMesh(tuple<vec3, int, int> lastIntPt, tuple<vec3, int, int> intPt, tuple<vec3, int, int> &nextIntPt, Edge* &leftCrossEdge, Edge* &rightCrossEdge, vec3 normal, int splitMode);
        void updateGhostSprings();
        void redistributeMass(); 

        //Updating the mesh
        void updateMesh(float dt, TimeIntegrationType integrationType = FWD_EULER);

        //Helper functions for computations with triangles
        bool isInside(Face* face, vec3 point);
        bool isInsidePos(Face* face, vec3 point);

        //Helper functions for interpolation
        vec3 getInitPosAtPoint(Particle* p);
        vec3 getInitPosAtEdge(Edge* e, vec3 pos);
        vec3 getInitPosAtFace(Face* f, vec3 pos);
        vec3 getPosAtPoint(Particle* p);
        vec3 getPosAtEdge(Edge* e, vec3 pos);
        vec3 getPosAtFace(Face* f, vec3 pos);
        vec3 getVelAtPos(Particle* p);
        vec3 getVelAtEdge(Edge* e, vec3 pos);
        vec3 getVelAtFace(Face* f, vec3 pos);

        int firstVertexIdx = -1;

    private:
        //Helper functions for checking intersection
        void ParticleOffsets(Plane plane);
        vector<int> IntersectingEdges();
        vector<tuple<vec3,int,int>> IntersectingVertices(vector<int> edges);

        //Internal updates of the mesh after every re triangulation operation 
        void resetForce();
        void assignInitialState();

        //Solver for the system of equations
        void solveFwdEuler(float dt);
        void solveBwdEuler(float dt);

};

#endif