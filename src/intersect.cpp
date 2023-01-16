#include "../include/intersect.hpp"

vec3 closestPtOnLine(const vec3& a, const vec3& b, const vec3& c){
    vec3 ab = b - a;
    vec3 ac = c - a;
    double t = ac.dot(ab.normalized());
    if(t < 0.0f){
        return a;
    }else if(t > ab.norm()){
        return b;
    }else{
        return a + t * ab.normalized();
    }
}

double triArea(const vec3& x, const vec3& y, const vec3& z){
    vec3 xy = y - x;
    vec3 xz = z - x;
    return 0.5 * xy.cross(xz).norm();
}

vec3 closestPtOnTri(const vec3& x, const vec3& y, const vec3& z, const vec3& c){
    vec3 xy = y - x; 
    vec3 xz = z - x;
    vec3 xc = c - x;
    vec3 n = xy.cross(xz).normalized();
    vec3 p = c + abs(xc.dot(n)) * n;

    double A = triArea(x, y, z);
    double Ax = triArea(p, y, z);
    double Ay = triArea(p, z, x);
    double Az = triArea(p, x, y);
    if(abs(Ax + Ay + Az - A) < 0.0001){
        //p lies inside the triangle
        return p;
    }else{
        //p lies outside the triangle
        //Closest triangle now lies on one of the edges
        vec3 pxy = closestPtOnLine(x, y, c);
        double minDist = (c - pxy).norm();
        vec3 minP = pxy;
        vec3 pxz = closestPtOnLine(x, z, c);
        if((c - pxz).norm() < minDist){
            minDist = (c - pxz).norm();
            minP = pxz;
        }
        vec3 pyz = closestPtOnLine(y, z, c);
        if((c - pyz).norm() < minDist){
            minDist = (c - pyz).norm();
            minP = pyz;
        }
        return minP;
    }
}

int intersectTri(const vec3& x, const vec3& y, const vec3& z, const vec3& a, const vec3& b, const vec3& c){
    //Convert vec3 into float arrays
    float xArr[3] = {x[0], x[1], x[2]};
    float yArr[3] = {y[0], y[1], y[2]};
    float zArr[3] = {z[0], z[1], z[2]};
    float aArr[3] = {a[0], a[1], a[2]};
    float bArr[3] = {b[0], b[1], b[2]};
    float cArr[3] = {c[0], c[1], c[2]};
    return NoDivTriTriIsect(xArr, yArr, zArr, aArr, bArr, cArr);
}