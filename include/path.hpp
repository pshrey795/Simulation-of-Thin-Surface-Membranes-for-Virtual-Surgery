#ifndef PATH_HPP
#define PATH_HPP

#define TIME_TO_PARAM_FACTOR 15.0

#include "curve.hpp"
using namespace std;

class Path {
    public:
        int size;
        vector<Curve*> curves; 
        int currentCurve = 0;
        float currParamVal = 0;
        bool isOver = false;
        vec3 lastPoint;
        vec3 lastTangent;
        Path();       
        void addCurve(vector<vec3> inputPts);
        void updatePath();  
};

#endif