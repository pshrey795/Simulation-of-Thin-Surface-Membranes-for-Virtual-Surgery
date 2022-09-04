#ifndef GUI_HPP
#define GUI_HPP

#include "common.hpp"

#include <chrono>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <thread>

class Window {
public:
    GLFWwindow *window;
    float lastFrameTime;
    int width, height;
    typedef void KeyPressCallback(int key);
    KeyPressCallback *keyPressed;
    Window();
    ~Window();
    void create(std::string name, int width, int height);
    void makeCurrent();
    void prepareDisplay();
    void updateDisplay();
    void processInput();
    bool shouldClose();
    void waitForNextFrame(float dt);
    vec2 mousePos();
    bool isMouseDown();
    bool isKeyPressed(int key);
    void onKeyPress(KeyPressCallback *keyPressed);
    static void errorCallback(int error, const char *description);
    static void resizeCallback(GLFWwindow* window, int width, int height);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
};

#endif
