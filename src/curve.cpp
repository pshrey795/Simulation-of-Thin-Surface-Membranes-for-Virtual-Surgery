#include "../include/curve.hpp"

Curve::Curve(vector<vec3> inputPts, int axis){
    this->size = inputPts.size();
    for(int i = 0; i < this->size; i++){
        this->controlPts.push_back(inputPts[i]);
    }
    this->axis = axis;
}

vec3 Curve::getPoint(float t){
    vec3 pt = vec3(0,0,0);
    int n = size - 1;
    for(int i=0;i<size;i++){
        float Ji = C(n,i) * power(t,i) * power(1.0f-t,n-i);
        pt += (controlPts[i] * Ji);
    }
    return pt;
}

vec3 Curve::getTangent(float t){
    vec3 tangent = vec3(0,0,0);
    int n = size - 1;
    for(int i=0;i<size;i++){
        float Ji = i * C(n,i) * power(t,i-1) * power(1.0f-t,n-i);
        Ji -= (size-i) * C(n,i) * power(t,i) * power(1.0f-t,n-i-1);
        tangent += controlPts[i] * Ji; 
    }
    return tangent;
}