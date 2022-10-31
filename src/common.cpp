#include "../include/common.hpp"

//Common Auxiliary Functions
bool double_eq(double a, double b){
    return (abs(a-b) <= 0.01f);
}

bool double_gt(double a, double b){
    return (a-b > 0.01f);
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

//Overload << operator for vec2, vec3
std::ostream& operator<<(std::ostream& os, const vec2& v){
    os << "(" << v[0] << ", " << v[1] << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const vec3& v){
    os << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
    return os;
}

//Eigen related functions for matrices and vectors
vecX matVecMult(const matX& A, const vecX& x){
    vecX res(A.rows());
    for(int i = 0; i < A.rows(); i++){
        for(int j = 0; j < A.cols(); j++){
            mat3 matVal = A(i,j);
            vec3 vecVal = x(j);
            res(i) = matVal*vecVal;
        }
    }
    return res;
}

vecX scalarMult(const vecX& x, float s){
    vecX res(x.rows());
    for(int i = 0; i < x.rows(); i++){
        res(i) = x(i)*s;
    }
    return res;
}