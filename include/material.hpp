#ifndef MAT_HPP
#define MAT_HPP

#include "common.hpp"

struct Material {
    vec3 albedo; 
    Material();
    Material(vec3 albedo);
};

#endif