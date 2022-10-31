#ifndef BLAS_WRAPPER_HPP
#define BLAS_WRAPPER_HPP

// Simple placeholder code for BLAS calls - replace with calls to a real BLAS library
// BLAS: Basic Linear Algebra Subprograms

#include <cmath>
#include <vector>

namespace BLAS{

// dot products ==============================================================

inline double dot(const std::vector<double> &x, const std::vector<double> &y){ 
    double sum = 0;
    for(unsigned int i = 0; i < x.size(); ++i){
        sum += x[i]*y[i];
    }
    return sum;
}

// inf-norm (maximum absolute value: index of max returned) ==================

inline int index_abs_max(const std::vector<double> &x){ 
    int maxind = 0;
    double maxvalue = 0;
    for(unsigned int i = 0; i < x.size(); ++i) {
        if(fabs(x[i]) > maxvalue) {
            maxvalue = fabs(x[i]);
            maxind = i;
        }
    }
    return maxind;
}

// inf-norm (maximum absolute value) =========================================

inline double abs_max(const std::vector<double> &x){ 
    return fabs(x[index_abs_max(x)]); 
}

// saxpy (y=alpha*x+y) =======================================================

inline void add_scaled(double alpha, const std::vector<double> &x, std::vector<double> &y)
{ 
    for(unsigned int i = 0; i < x.size(); ++i){
        y[i] += alpha*x[i];
    }
}

}

#endif
