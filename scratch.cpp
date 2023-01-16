#include "include/BVH.hpp"

int main(){
    vec3 x = vec3(0,0,0);
    vec3 y = vec3(4,0,0);
    vec3 z = vec3(0,3,0);
    vec3 a = vec3(-3,1.5,0);
    vec3 b = vec3(-1,1.5,3);
    vec3 c = vec3(-1,1.5,-3);
    int hitRecord = intersectTri(x,y,z,a,b,c);
    cout << hitRecord << endl;
    return 0;
}