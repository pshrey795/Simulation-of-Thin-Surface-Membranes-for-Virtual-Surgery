#include "include/model.hpp"

using namespace std;

Window window;
Camera camera;
Lighting lighting;

//Scene Objects
Model model;
Model model2;

//Rendering Modes
int mode = 0;
int drawMode = 0;
int splitMode = 0;

float dt = 1/60.0f;
float t = 0;
bool paused = false;

void drawEnv(){
    setColor(vec3(0.0f, 0.0f, 0.0f));
    drawArrow(vec3(-16.0f,-16.0f,-25.0f), vec3(12.0f, 12.0f, 25.0f), 0.3f);
    drawArrow(vec3(16.0f,16.0f,-25.0f), vec3(-12.0f, -12.0f, 25.0f), 0.3f);
    drawArrow(vec3(-16.0f,16.0f,-25.0f), vec3(12.0f, -12.0f, 25.0f), 0.3f);
    drawArrow(vec3(16.0f,-16.0f,-25.0f), vec3(-12.0f, 12.0f, 25.0f), 0.3f);
    setColor(vec3(0.3f, 0.5f, 0.7f));
    drawQuad(vec3(-25.0f,-25.0f,-25.0f), vec3(25.0f, -25.0f, -25.0f), vec3(25.0f, 25.0f, -25.0f), vec3(-25.0f, 25.0f, -25.0f));
}

void drawWorld() {
    camera.apply(window);
    lighting.apply();
    clear(vec3(0.5,0.7,0.9));
    drawEnv();
    setColor(vec3(0.7,0.7,0.7));
    model.renderModel();
    model2.renderModel();
    setColor(vec3(0,0,0));
}

void update(float dt) {
    t += dt;
}

void keyPressed(int key) {
    if (key == GLFW_KEY_SPACE)
        paused = !paused;
}

void processInput(int argc, char** argv){
    if(argc > 1){
        mode = atoi(argv[1]);
    }
    if(argc > 2){
        drawMode = atoi(argv[2]);
    }
    if(argc > 3){
        splitMode = atoi(argv[3]);
    }
    if(mode){
        model.activateRefMesh();
    }
    model.setDrawMode(drawMode);
    model.setSplitMode(splitMode);
}

int main(int argc, char **argv) {
    window.create("Test Window", 2560, 1440);
    window.onKeyPress(keyPressed);
    camera.lookAt(vec3(0.0f,-45.0f,45.0f), vec3(0.0f,0.0f,-10.0f));
    lighting.createDefault();
    model2 = Model("objects/sphere.obj");

    processInput(argc, argv);

    while (!window.shouldClose()) {
        camera.processInput(window);
        model.processInput(window);
        model2.processInput(window);
        if (!paused){
            update(dt);
            model.update(dt);
            model2.update(dt);
        }
        window.prepareDisplay();
        drawWorld();
        window.updateDisplay();
        window.waitForNextFrame(dt);
    }

    //Post Simulation debugging 
    // model.printMeshInfo();
}
