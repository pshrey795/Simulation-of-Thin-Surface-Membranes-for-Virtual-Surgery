#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "rigid_body.hpp"
#include "deformable_body.hpp"

class Collider {

    public:
        Collider();
        void detectCollision(DeformableBody& membrane, RigidBody& instrument);
        void updateMesh(DeformableBody& membrane, vector<int>& intersectingEdges);
        void resetCollider();

    private:
        //A crack tip of a fracture is the crack front(here, set of vertices) where are edges are more susceptible to cuts
        unordered_set<int> crackTip;
        float normalThreshold;
        float crackTipThreshold;

};

#endif
