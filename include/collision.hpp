#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "rigid_body.hpp"
#include "deformable_body.hpp"

void detectCollision(DeformableBody& membrane, RigidBody& instrument);
void updateMesh(DeformableBody& membrane, vector<int>& intersectingEdges);

#endif
