#include "../include/collision.hpp"

bool deq(double a, double b){
    return abs(a - b) < 1e-6;
}

double penalty(double x){
    return 100.0f * x;
}

void extrapolateLine(Particle* x, Particle* y, vec3 p, vec3 val, int responseMode){
    vec3 a = x->position; vec3 b = y->position;
    double L = (b - a).norm();
    double L1 = (p - a).norm();
    double L2 = (p - b).norm();
    val = val / 2.0f; 
    double u_inv = L / L1;
    double v_inv = L / L2;
    if(responseMode){
        x->externalForce += val * u_inv;
        y->externalForce += val * v_inv;
    }else{
        x->constraint.isActive = true;
        y->constraint.isActive = true;
        x->constraint.velocity += val * u_inv;
        y->constraint.velocity += val * v_inv;
    }
}

void extrapolateTri(Particle* x, Particle* y, Particle* z, vec3 p, vec3 val, int responseMode){
    vec3 a = x->position; vec3 b = y->position; vec3 c = z->position;
    double A = triArea(a, b, c);
    double A1 = triArea(p, b, c);
    double A2 = triArea(p, c, a);
    double A3 = triArea(p, a, b);
    if(deq(A1, 0.0f)){
        if(deq(A2, 0.0f)){
            //p is Particle z
            if(responseMode){
                z->externalForce += val;
            }else{
                z->constraint.isActive = true;
                z->constraint.velocity += val;
            }
        }else{
            if(deq(A3, 0.0f)){
                //p is Particle y
                if(responseMode){
                    y->externalForce += val;
                }else{
                    y->constraint.isActive = true;
                    y->constraint.velocity += val;
                }
            }else{
                //p is on edge yz
                extrapolateLine(y, z, p, val, responseMode);
            }
        }
    }else{
        if(deq(A2, 0.0f)){
            if(deq(A3, 0.0f)){
                //p is Particle x
                if(responseMode){
                    x->externalForce += val;
                }else{
                    x->constraint.isActive = true;
                    x->constraint.velocity += val;
                }
            }else{
                //p is on edge xz
                extrapolateLine(x, z, p, val, responseMode);
            }
        }else{
            if(deq(A3, 0.0f)){
                //p is on edge xy
                extrapolateLine(x, y, p, val, responseMode);
            }else{
                double u_inv = A/A1;
                double v_inv = A/A2;
                double w_inv = A/A3;
                val = val / 3.0f; 
                if(responseMode){
                    x->externalForce += val * u_inv;
                    y->externalForce += val * v_inv;
                    z->externalForce += val * w_inv;
                }else{
                    x->constraint.isActive = true;
                    y->constraint.isActive = true;
                    z->constraint.isActive = true;
                    x->constraint.velocity += val * u_inv;
                    y->constraint.velocity += val * v_inv;
                    z->constraint.velocity += val * w_inv;
                }
            }
        }
    }
}

void detectCollision(DeformableBody& membrane, RigidBody& instrument, vector<vec3>& intersectionPoints){
    //Reset all constraints on the membrane
    for(int i = 0; i < membrane.mesh->particle_list.size(); i++){
        membrane.mesh->particle_list[i]->externalForce = vec3(0.0f, 0.0f, 0.0f); 
        if(membrane.mesh->constraints.find(i) == membrane.mesh->constraints.end()){
            membrane.mesh->particle_list[i]->constraint.isActive = false;
            membrane.mesh->particle_list[i]->constraint.velocity = vec3(0.0f, 0.0f, 0.0f);
        }
    }
    
    //Membrane represented as a list of primitives(triangles)
    //Rigid Body representation dependant on the type of object
    if(instrument.collisionMode){
        //Rigid Body approximated by a sphere

        //1. Particle Sphere Intersection
        //Collision Response on particles
        // for(int i = 0; i < membrane.mesh->particle_list.size(); i++){
        //     vec3 pos = membrane.mesh->particle_list[i]->position;
        //     vec3 centre = instrument.sphere.centre;
        //     double length = (pos - centre).norm();
        //     if(length < instrument.sphere.radius){
        //         //Collision Response
        //         if(instrument.responseMode){
        //             //2. Penalty Forces
        //             vec3 unitVec = (pos - centre).normalized();
        //             double penaltyForce = penalty(instrument.sphere.radius - length);
        //             membrane.mesh->particle_list[i]->externalForce += penaltyForce * unitVec;
        //         }else{
        //             //1. Velocity Constraints
        //             membrane.mesh->particle_list[i]->constraint.isActive = true;
        //             vec3 unitVec = (pos - centre).normalized();
        //             vec3 rigidVel = instrument.velocity;
        //             double cosine = (rigidVel.normalized()).dot(unitVec);
        //             vec3 vel = rigidVel.norm() * cosine * unitVec;
        //             membrane.mesh->particle_list[i]->constraint.velocity = rigidVel;
        //         }        
        //     }else{
        //         if(membrane.mesh->constraints.find(i) == membrane.mesh->constraints.end()){
        //             membrane.mesh->particle_list[i]->constraint.isActive = false;
        //             membrane.mesh->particle_list[i]->constraint.velocity = vec3(0.0f, 0.0f, 0.0f);
        //         }
        //     }
        // }

        //2. Triangle Sphere Intersection
        //Collision Response on faces
        intersectionPoints.clear();
        for(int i = 0; i < membrane.mesh->face_list.size(); i++){
            //First find the closest point on triangle to the sphere centre
            vec3 x = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[0]]->position;
            vec3 y = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[1]]->position;
            vec3 z = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[2]]->position;
            vec3 normal = (y - x).cross(z - x).normalized();
            vec3 centre = instrument.sphere.centre;
            vec3 closest = closestPtOnTri(x, y, z, centre);
            double length = (closest - centre).norm();
            vec3 closestUnitVec = (closest - centre).normalized();
            if(length < instrument.sphere.radius){
                //Track triangles in direct collision
                double xLength = (x - centre).norm() - instrument.sphere.radius;
                double yLength = (y - centre).norm() - instrument.sphere.radius;
                double zLength = (z - centre).norm() - instrument.sphere.radius;
                if(xLength >= 0.0f || yLength >= 0.0f || zLength >= 0.0f){
                    membrane.mesh->face_list[i]->color = vec3(0.0f, 1.0f, 0.0f);
                    membrane.mesh->face_list[i]->reMeshed = true;
                }else{
                    membrane.mesh->face_list[i]->color = vec3(1.0f, 0.0f, 0.0f);
                    membrane.mesh->face_list[i]->reMeshed = false;
                }
                //Collision Response
                if(instrument.responseMode){
                    //2. Penalty Force
                    double penaltyForce = penalty(instrument.sphere.radius - length);
                    vec3 forceVal = penaltyForce * closestUnitVec;
                    // extrapolateTri(membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[0]], 
                    //                membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[1]], 
                    //                membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[2]], 
                    //                closest, forceVal, instrument.responseMode);
                }else{
                    //1. Velocity constraints
                    //Finding the component of velocity in the direction of the closest point
                    vec3 rigidVel = instrument.velocity;
                    double cosine = (rigidVel.normalized()).dot(closestUnitVec);
                    vec3 vel = rigidVel.norm() * cosine * closestUnitVec;
                    //This is the velocity of closest point
                    //Now, we need to find the velocity of the particles surrounding it
                    //We use extrapolation for that
                    extrapolateTri(membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[0]], 
                                   membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[1]], 
                                   membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[2]], 
                                   closest, vel, instrument.responseMode);
                }
            }else{
                membrane.mesh->face_list[i]->color = vec3(1.0f, 0.0f, 0.0f);
            }
        }
    }else{
        //Rigid Body represented as a BVH
        // //1. Triangle BVH intersection
        // for(int i = 0; i < membrane.mesh->face_list.size(); i++){
        //     vec3 x = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[0]]->position;
        //     vec3 y = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[1]]->position;
        //     vec3 z = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[2]]->position;
        //     bool isIntersecting = instrument.intersectBVH(x, y, z);
        //     if(isIntersecting){
        //         membrane.mesh->face_list[i]->color = vec3(0.0f, 1.0f, 0.0f);
        //     }else{
        //         membrane.mesh->face_list[i]->color = vec3(1.0f, 0.0f, 0.0f);
        //     }
        // }

        // 2. Triangle Triangle intersection 
        // For debugging
        for(int i = 0; i < membrane.mesh->face_list.size(); i++){
            vec3 x = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[0]]->position;
            vec3 y = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[1]]->position;
            vec3 z = membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[2]]->position;
            vec3 normal = (y - x).cross(z - x).normalized();
            vec3 centre = instrument.sphere.centre;
            vec3 closest = closestPtOnTri(x, y, z, centre);
            double length = (closest - centre).norm();
            vec3 closestUnitVec = (closest - centre).normalized();
            int result;
            for(int j = 0; j < instrument.faces.size(); j++){
                vec3 a = instrument.vertices[instrument.faces[j].v1];
                vec3 b = instrument.vertices[instrument.faces[j].v2];
                vec3 c = instrument.vertices[instrument.faces[j].v3];
                result = intersectTri(x, y, z, a, b, c);
                if(result){
                    break;
                }
            }
            if(result){
                membrane.mesh->face_list[i]->color = vec3(0.0f, 1.0f, 0.0f);
                // double penaltyForce = penalty(instrument.sphere.radius - length);
                // vec3 forceVal = penaltyForce * closestUnitVec;
                // extrapolateTri(membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[0]], 
                //                 membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[1]], 
                //                 membrane.mesh->particle_list[membrane.mesh->face_list[i]->indices[2]], 
                //                 closest, forceVal, instrument.responseMode);
            }else{
                membrane.mesh->face_list[i]->color = vec3(1.0f, 0.0f, 0.0f);
            }
        }
    }
}