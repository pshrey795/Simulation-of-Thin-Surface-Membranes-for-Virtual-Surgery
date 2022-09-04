#ifndef CURVE_HPP
#define CURVE_CPP

#include "common.hpp"
#include<bits/stdc++.h>

using namespace std;

class Curve {

    private:
        int size;
        vector<vec3> controlPts;
        int axis;

    public:
        Curve(vector<vec3> inputPts, int axis);
        vec3 getPoint(float t);
        vec3 getTangent(float t);

};

#endif