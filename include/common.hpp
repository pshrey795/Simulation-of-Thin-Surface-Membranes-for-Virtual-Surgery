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
typedef Eigen::Matrix<double, 3, 3> mat3;

using namespace Eigen;

//Common Auxiliary Functions
bool double_eq(double a, double b);
bool double_gt(double a, double b);
long long factorial(int n);
double C(int n, int r);
double power(double x, int n);

#endif
