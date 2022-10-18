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

#define DEFAULT_MASS 1.0f
#define DEFAULT_DENISTY 0.35f
#define DEFAULT_STIFFNESS 10.0f
#define DEFAULT_DAMPING 5.0f
#define EPSILON 0.08
#define DELTA 0.01
#define GRAVITY vec3(0.0f, 0.0f, -9.8f)

typedef Eigen::Vector2f vec2;
typedef Eigen::Vector3f vec3;
typedef Eigen::Vector4f vec4;
typedef Eigen::Matrix<double, 3, 3> mat3;

using namespace Eigen;

//Common Auxiliary Functions
bool double_eq(double a, double b);
bool double_gt(double a, double b);
long long factorial(int n);
double C(int n, int r);
double power(double x, int n);

//Overload << operator for vec2, vec3
std::ostream& operator<<(std::ostream& os, const vec2& v);
std::ostream& operator<<(std::ostream& os, const vec3& v);

#endif
