#include "include/collision.hpp"

using namespace std;

Window window;
Camera camera;
Lighting lighting;

//Scene Objects
DeformableBody membrane; 
RigidBody instrument;

//Rendering Modes
int mode = 0;
int drawMode = 0;
int splitMode = 0;

//Debugging
vector<vec3> intersectionPoints;

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
    setPointSize(10.0f);
    setColor(vec3(0.0f, 0.0f, 1.0f));
    for(auto v : intersectionPoints){
        drawPoint(v);
    }
}

void drawWorld() {
    camera.apply(window);
    lighting.apply();
    clear(vec3(0.5,0.7,0.9));
    drawEnv();
    setColor(vec3(0.7,0.7,0.7));
    membrane.renderMesh();
    instrument.renderMesh();
    // drawDebug();
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
        membrane.drawRefMesh = true;
    }
    membrane.drawMode = drawMode;
    membrane.splitMode = splitMode;
}

int main(int argc, char **argv) {
    window.create("Test Window", 2560, 1440);
    window.onKeyPress(keyPressed);
    camera.lookAt(vec3(0.0f,-45.0f,45.0f), vec3(0.0f,0.0f,-10.0f));
    lighting.createDefault();
    instrument = RigidBody("objects/sphere.obj");
    processInput(argc, argv);
    while (!window.shouldClose()) {
        camera.processInput(window);
        membrane.processInput(window);
        instrument.processInput(window);
        update(dt);
        membrane.update(dt);
        instrument.update(dt);
        window.prepareDisplay();
        drawWorld();
        detectCollision(membrane, instrument, intersectionPoints);
        window.updateDisplay();
        window.waitForNextFrame(dt);
    }
}
