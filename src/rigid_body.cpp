#include "../include/rigid_body.hpp"

RigidBody::RigidBody(){
    debug = false;
}

RigidBody::RigidBody(vector<vec3> Vertices, vector<unsigned int> Indices){
    debug = false;
    vertices = Vertices;
    indices = Indices;
    velocity = vec3(0,0,-0.75f);
}

void RigidBody::update(float dt){
    if(activatePhysics){
        for(int i = 0;i < vertices.size();i++){
            vertices[i] += velocity * dt;
        }
    }
}

void RigidBody::processInput(Window &window){
    GLFWwindow* win = window.window;
    if(glfwGetKey(win, GLFW_KEY_O) == GLFW_PRESS){
        if(!activatePhysics){
            activatePhysics = true;
        }
    }
}

void RigidBody::renderMesh(){
    int m = indices.size();

    //Drawing faces
    for(int i = 0; i < m; i+=3){
        vec3 v1 = vertices[indices[i]];
        vec3 v2 = vertices[indices[i+1]];
        vec3 v3 = vertices[indices[i+2]];
        setColor(mat.albedo);
        drawTri(v1, v2, v3);
    }
}

void RigidBody::printMeshInfo(){

}

bool RigidBody::checkSanity(){
    return true;
}