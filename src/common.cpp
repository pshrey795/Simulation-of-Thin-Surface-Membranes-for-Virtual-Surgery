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