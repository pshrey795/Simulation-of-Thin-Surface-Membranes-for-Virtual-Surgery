#include "include/debug.hpp"

int main(){
    matX A(1,1);
    A(0,0) = 3 * mat3::Identity();
    vecX x(1);
    x(0) = vec3(1,2,3);
    cout << explodeMatrix(A) << endl;
    cout << explodeVector(x) << endl;
    cout << explodeVector(matVecMult(A,x)) << endl;
}