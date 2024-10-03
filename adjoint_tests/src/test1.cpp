#include <iostream>
#include <vector>          // standard vector
#include <cppad/cppad.hpp>

#define _USE_MATH_DEFINES
#include "math.h"
using CppAD::AD; //use AD as abbreviation for CppAD::AD

int main() {

    std::cout <<"test " << std::endl;
   // Create state(domain) space vector
    size_t n = 1;               // number of domain space variables
    std::vector< AD<double> > x(n); // vector of domain space variables
    x[0] = 0; // value at which function is recorded

    //declare independent variables and start recording operation sequence
    CppAD::Independent(x);


    // range output(range) space vector
    size_t m = 1;               // number of ranges space variables
    std::vector< AD<double> > rhs(m); // vector of ranges space variables


    //records operation that compute rhs (create the function)
    rhs[0] = pow(x[0],2);
    // rhs[0] = x[0];
    // rhs[0] *= x[0];

    // store operation sequence in f: X -> Y and stop recording (x = states/independent vars -> Y = outputs)
   CppAD::ADFun<double> f(x, rhs);

    
    std::vector<double> x_value(n);
    x_value[0] = 4.0;
    std::cout << "f(x=4) = " << f.Forward(0, x_value)[0] << std::endl; // Get y value
    std::cout << "f'(x=4) = " << f.Forward(1, x_value)[0] << std::endl; // Get y value


    // CppAD::ADFun<ADdouble> f(x, y); //f(independent vars, dependent vars)

    // std::cout << f.Forward(0, 3);

    
    return 0;
}
