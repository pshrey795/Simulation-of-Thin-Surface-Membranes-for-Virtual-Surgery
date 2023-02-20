#include "../include/gui.hpp"

Window::Window():
    window(NULL),
    lastFrameTime(0),
    width(0), height(0),
    keyPressed(NULL), debug(false) {
}

void Window::terminate(){
    if (!window)
        return;
    glfwDestroyWindow(window);
}

void Window::create(std::string name, int width, int height) {
    glfwSetErrorCallback(errorCallback);
    if (!glfwInit())
        exit(1);
    window = glfwCreateWindow(width, height, name.c_str(), NULL, NULL);
    makeCurrent();
    glfwSetWindowUserPointer(window, this);
    glfwSetFramebufferSizeCallback(window, resizeCallback);
    resizeCallback(window, width, height);
    glfwSetKeyCallback(window, keyCallback);
}

void Window::makeCurrent() {
    glfwMakeContextCurrent(window);
}

void Window::prepareDisplay() {
    glEnable(GL_DEPTH_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_NORMALIZE);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Window::updateDisplay() {
    glfwSwapBuffers(window);
}

bool Window::shouldClose() {
    return glfwWindowShouldClose(window);
}

void Window::waitForNextFrame(float dt) {
    glfwPollEvents();
    float waitTime = lastFrameTime + dt - glfwGetTime();
    if (waitTime > 0)
        std::this_thread::sleep_for(std::chrono::microseconds(int(waitTime*1e6)));
    lastFrameTime = glfwGetTime();
}

vec2 Window::mousePos() {
    double x, y;
    glfwGetCursorPos(window, &x, &y);
    return vec2(x,y);
}

bool Window::isMouseDown() {
    return (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
}

bool Window::isKeyPressed(int key) {
    return (glfwGetKey(window, key) == GLFW_PRESS);
}

void Window::onKeyPress(KeyPressCallback *keyPressed) {
    this->keyPressed = keyPressed;
}

void Window::errorCallback(int error, const char* description) {
    std::cerr << "Error: " << description << std::endl;
}

void Window::resizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    Window *w = (Window*)glfwGetWindowUserPointer(window);
    w->width = width;
    w->height = height;
}

void Window::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    Window *w = (Window*)glfwGetWindowUserPointer(window);
    if (w->keyPressed && action == GLFW_PRESS)
        w->keyPressed(key);
}