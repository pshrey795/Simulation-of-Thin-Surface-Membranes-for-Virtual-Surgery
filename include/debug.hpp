#ifndef DEBUG_HPP
#define DEBUG_HPP

#include "common.hpp"
#include "gui.hpp"

//Debug Files
extern string debugFile;
extern ofstream debugStream;

//Overload << operator for vec2, vec3
std::ostream& operator<<(std::ostream& os, const vec2& v);
std::ostream& operator<<(std::ostream& os, const vec3& v);
std::ostream& operator<<(std::ostream& os, const vecXf& A);
std::ostream& operator<<(std::ostream& os, const mat3& A);
std::ostream& operator<<(std::ostream& os, const matXf& A);

#endif
