#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "common.hpp"
#include "gui.hpp"

class Camera {
public:
    vec3 target;
    float yaw, pitch, dist;
    float fovy, moveSpeed, zoomSpeed, turnSpeed;
    float lastInputTime;
    bool mousePressed;
    vec2 lastMousePos;
    Camera();
    void lookAt(vec3 eye, vec3 target);
    void processInput(Window &window);
    void apply(Window &window);
    
protected:
    vec3 getForwardVector();
    vec3 getRightVector();
    vec3 getUpVector();
};

#endif
