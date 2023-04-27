#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "rigid_body.hpp"
#include "deformable_body.hpp"

class Collider {

    public:
        Collider();
        void detectCollision(DeformableBody& membrane, RigidBody& instrument);
        void updateMesh(DeformableBody& membrane, vector<int>& intersectingEdges, int crackMode = 0);

    private:
        float normalThreshold;
        float crackTipThreshold;

};

#endif
