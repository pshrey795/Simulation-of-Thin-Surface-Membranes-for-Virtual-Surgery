#ifndef COMMON_HPP
#define COMMON_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <bits/stdc++.h>

#if __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GLFW/glfw3.h>
#endif

#define DEFAULT_MASS 1.0f
#define DEFAULT_DENISTY 0.35f
#define DEFAULT_STIFFNESS 10.0f
#define DEFAULT_DAMPING 5.0f
#define EPSILON 0.03
#define DELTA 0.1
#define MIN_DIFF 0.01
#define GRAVITY vec3(0.0f, 0.0f, -9.8f)

using namespace std;
using namespace Eigen;

typedef Eigen::Vector2f vec2;
typedef Eigen::Vector3f vec3;
typedef Eigen::Vector4f vec4;
typedef Eigen::Matrix<vec3, Dynamic, 1> vecX;
typedef Eigen::Matrix<float, Dynamic, 1> vecXf;
typedef Eigen::Matrix<float, 3, 3> mat3;
typedef Eigen::Matrix<float, Dynamic, Dynamic> matXf;
typedef Eigen::Matrix<mat3, Dynamic, Dynamic> matX;

//Common Auxiliary Functions
bool double_eq(double a, double b);
bool double_gt(double a, double b);
long long factorial(int n);
double C(int n, int r);
double power(double x, int n);

//Overload << operator for vec2, vec3
std::ostream& operator<<(std::ostream& os, const vec2& v);
std::ostream& operator<<(std::ostream& os, const vec3& v);
std::ostream& operator<<(std::ostream& os, const vecXf& A);
std::ostream& operator<<(std::ostream& os, const mat3& A);
std::ostream& operator<<(std::ostream& os, const matXf& A);

//Eigen related functions for matrices and vectors
vecX matVecMult(const matX& A, const vecX& x);
vecX scalarMult(const vecX& x, float s);
matXf explodeMatrix(const matX& A);
vecXf explodeVector(const vecX& x);
vecX compressVector(const vecXf& x);

//Debug Files
extern string debugFile;
extern ofstream debugStream;

#endif
