#ifndef DRAW_HPP
#define DRAW_HPP

#include "common.hpp"

//Common OpenGL based draw functions 

void clear(vec3 c);
void setColor(vec3 c);
void setColor(vec4 c);
void setPointSize(float s);
void drawPoint(vec3 x);
void setLineWidth(float w);
void drawLine(vec3 x0, vec3 x1);
void drawArrow(vec3 base, vec3 vector, float thick);
void drawAxes();
void drawTri(vec3 x0, vec3 x1, vec3 x2);
void drawQuad(vec3 x0, vec3 x1, vec3 x2, vec3 x3);

// for smooth shading, provide surface normal at each vertex
void drawTri(vec3 x0, vec3 x1, vec3 x2, vec3 n0, vec3 n1, vec3 n2);
void drawQuad(vec3 x0, vec3 x1, vec3 x2, vec3 x3, vec3 n0, vec3 n1, vec3 n2, vec3 n3);
void drawBox(vec3 xmin, vec3 xmax);
void drawSphere(vec3 center, float radius);
    
#endif
