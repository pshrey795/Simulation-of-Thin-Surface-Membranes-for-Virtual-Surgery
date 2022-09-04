#ifndef LIGHTING_HPP
#define LIGHTING_HPP

#include "common.hpp"
#include "gui.hpp"

#include <vector>

class Lighting {
public:
    void addLight(vec3 dir, vec3 color);
    void createDefault();
    void apply();
protected:
    struct Light {
        vec3 dir, color;
        Light (vec3 dir, vec3 color): dir(dir), color(color) {}
    };
    std::vector<Light> lights;
};

#endif
