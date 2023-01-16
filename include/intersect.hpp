#ifndef INTERSECT_HPP
#define INTERSECT_HPP

#include "common.hpp"
#include "TriTri.hpp"

//Intersection functions
double triArea(const vec3& x, const vec3& y, const vec3& z);
vec3 closestPtOnLine(const vec3& a, const vec3& b, const vec3& c);
vec3 closestPtOnTri(const vec3& x, const vec3& y, const vec3& z, const vec3& c);
int intersectTri(const vec3& x, const vec3& y, const vec3& z, const vec3& a, const vec3& b, const vec3& c);

#endif