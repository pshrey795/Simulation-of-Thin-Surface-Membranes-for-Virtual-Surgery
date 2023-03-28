#include "include/collision.hpp"

using namespace std;

Window window;
Window debugWindow;
Camera camera;
Camera debugCamera;
Lighting lighting;

//Scene Objects
DeformableBody membrane; 
RigidBody instrument;

//Rendering Modes
int mode = 0;
int drawMode = 0;

//Debugging
vector<int> intersectingEdges;

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

void drawDebug(){
    debugCamera.apply(debugWindow);
    lighting.apply();
    clear(vec3(0.5,0.7,0.9));
    setColor(vec3(0.7,0.7,0.7));
    membrane.renderDebugMesh();
    setColor(vec3(0,0,0));
}

void drawWorld(){
    camera.apply(window);
    lighting.apply();
    clear(vec3(0.5,0.7,0.9));
    drawEnv();
    setColor(vec3(0.7,0.7,0.7));
    membrane.renderMesh();
    instrument.renderMesh();
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
        drawMode = atoi(argv[1]);
    }
    membrane.drawMode = drawMode;
}

int main(int argc, char **argv) {
    //Creating windows
    window.create("Simulation Window", 960, 540);
    window.onKeyPress(keyPressed);
    debugWindow.create("Debug Window", 960, 540);
    debugWindow.onKeyPress(keyPressed);
    debugWindow.debug = true;

    //Creating simulation objects
    camera.lookAt(vec3(0.0f,-20.0f,25.0f), vec3(0.0f,0.0f,-0.0f));
    debugCamera.lookAt(vec3(0.0f,-10.0f,15.0f), vec3(0.0f,0.0f,-0.0f));
    lighting.createDefault();
    instrument = RigidBody("objects/sphere.obj");
    processInput(argc, argv);

    while(!window.shouldClose() && !debugWindow.shouldClose()){
        //Rendering the main simulation
        window.makeCurrent();
        camera.processInput(window);
        membrane.processInput(window);
        instrument.processInput(window);
        window.prepareDisplay();
        drawWorld();
        window.updateDisplay();
        window.waitForNextFrame(dt);

        //Rendering debug visualization
        debugWindow.makeCurrent();
        debugCamera.processInput(debugWindow);
        membrane.processInput(debugWindow);
        instrument.processInput(debugWindow);
        debugWindow.prepareDisplay();
        drawDebug();
        debugWindow.updateDisplay();
        debugWindow.waitForNextFrame(dt);

        //Simulation update
        update(dt);
        membrane.update(dt);
        instrument.update(dt);
        detectCollision(membrane, instrument);
        if(mode){
            updateMesh(membrane, intersectingEdges);
        }
    }
    window.terminate();
    debugWindow.terminate();
    glfwTerminate();
}
