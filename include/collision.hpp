#ifndef COLLISION_HPP
#define COLLISION_HPP

#include "rigid_body.hpp"
#include "deformable_body.hpp"

void detectCollision(DeformableBody& membrane, RigidBody& instrument, vector<vec3>& intersectionPoints);

#endif
