#include "include/camera.hpp"
#include "include/draw.hpp"
#include "include/gui.hpp"
#include "include/lighting.hpp"
#include "include/mesh.hpp"

#include <bits/stdc++.h>

using namespace std;

Window window;
Camera camera;
Lighting lighting;
Mesh model;

float dt = 1/60.0;
float t = 0;
bool paused = false;

void drawWorld() {
    camera.apply(window);
    lighting.apply();
    clear(vec3(0.5,0.7,0.9));
    setColor(vec3(0.7,0.7,0.7));
    model.renderMesh();
    setColor(vec3(0,0,0));
}

void update(float dt) {
    t += dt;
}

void keyPressed(int key) {
    if (key == GLFW_KEY_SPACE)
        paused = !paused;
}

int main(int argc, char **argv) {
    window.create("Test Window", 1920, 1080);
    window.onKeyPress(keyPressed);
    camera.lookAt(vec3(12.5,12.5,50), vec3(12.5,12.5,0));
    lighting.createDefault();

    while (!window.shouldClose()) {
        camera.processInput(window);
        model.processInput(window);
        if (!paused){
            update(dt);
            model.update(dt);
        }
        window.prepareDisplay();
        drawWorld();
        window.updateDisplay();
        window.waitForNextFrame(dt);
    }
}
