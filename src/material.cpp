#include "../include/material.hpp"

Material::Material(){
    albedo = vec3(1.0f, 0.0f, 0.0f);
}

Material::Material(vec3 albedo){
    this->albedo = albedo;
}