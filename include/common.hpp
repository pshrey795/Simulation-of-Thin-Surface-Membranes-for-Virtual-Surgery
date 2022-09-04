#ifndef COMMON_HPP
#define COMMON_HPP

#include <eigen3/Eigen/Dense>
#include <bits/stdc++.h>

#if __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

typedef Eigen::Vector2f vec2;
typedef Eigen::Vector3f vec3;
typedef Eigen::Matrix<float, 3, 3> mat3;

using namespace Eigen;

//Common Auxiliary Functions
bool double_eq(double a, double b){
    return (abs(a-b) <= 0.01f);
}

long long factorial(int n){
    long long res = 1;
    for(int i=1;i<=n;i++){
        res *= (long long)i;
    }
    return res;
}

double C(int n, int r){
    return (double)factorial(n)/((double)(factorial(r)*factorial(n-r)));
}

double power(double x, int n){
    if(n==0){
        return 1;
    }else{
        return pow(x,n);
    }
}

#endif
