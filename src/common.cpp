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

//Eigen related functions for matrices and vectors
matX matMult(const matX& A, const matX& B){
    matX res(A.rows(), B.cols());
    for(int i = 0; i < A.rows(); i++){
        for(int j = 0; j < B.cols(); j++){
            mat3 matVal = mat3::Zero();
            for(int k = 0; k < A.cols(); k++){
                matVal += A(i,k)*B(k,j);
            }
            res(i,j) = matVal;
        }
    }
    return res;
}

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

//Convert matrix of blocks to a matrix of floats
matXf explodeMatrix(const matX& A){
    matXf res(A.rows()*3, A.cols()*3);
    for(int i = 0; i < A.rows(); i++){
        for(int j = 0; j < A.cols(); j++){
            mat3 matVal = A(i,j);
            for(int k = 0; k < 3; k++){
                for(int l = 0; l < 3; l++){
                    res(i*3+k, j*3+l) = matVal(k,l);
                }
            }
        }
    }
    return res;
}
vecXf explodeVector(const vecX& x){
    vecXf res(x.rows()*3);
    for(int i = 0; i < x.rows(); i++){
        vec3 vecVal = x(i);
        for(int j = 0; j < 3; j++){
            res(i*3+j) = vecVal(j);
        }
    }
    return res;
}

//Convert matrix of floats to a matrix of blocks
vecX compressVector(const vecXf& x){
    assert(x.rows()%3 == 0);
    vecX res(x.rows()/3);
    for(int i = 0; i < x.rows()/3; i++){
        vec3 vecVal;
        for(int j = 0; j < 3; j++){
            vecVal(j) = x(i*3+j);
        }
        res(i) = vecVal;
    }
    return res;
}
