#include "../include/debug.hpp"

//Debug Files
string debugFile = "Results/debug.txt";
ofstream debugStream = ofstream(debugFile);

//Overload << operator for vec2, vec3
std::ostream& operator<<(std::ostream& os, const vec2& v){
    os << "(" << v[0] << ", " << v[1] << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const vec3& v){
    os << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, const vecXf& A){
    for(int i=0;i<A.rows();i++){
        os << A(i) << endl;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const mat3& A){
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            os << A(i,j) << " ";
        }
        os << endl;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const matXf& A){
    for(int i=0;i<A.rows();i++){
        for(int j=0;j<A.cols();j++){
            os << A(i,j) << " ";
        }
        os << endl;
    }
    return os;
}