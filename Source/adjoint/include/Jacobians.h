#pragma once

#include <iostream>
#include <vector>          // standard vector
#include <array>
#include <cppad/cppad.hpp>

#define _USE_MATH_DEFINES
#include "math.h"
using CppAD::AD; //use AD as abbreviation for CppAD::AD

#include <UAV.h>



class Jacobians{
public:
    static const size_t r = 4; // number of param-domain space variables (b)
    static const size_t n = 4; // number of state-domain space variables
    static const size_t nc = 2; //num of conntrol variables
    static const size_t m = 2; // number of ranges space variables

    Jacobians(UAV uav, std::array<double, 6> p, std::array<double, r> b, double ts, double tf);

    CppAD::ADFun<double> F_u(std::array<double, nc> c);
    CppAD::ADFun<double> J_u();
    CppAD::ADFun<double> F_b();

    static void printJacobian(const std::vector<double>& jac, int rows, int columns);

private:
    std::array<double, r> b; 
    std::array<double, 6> p; //coefficients from engine map
    CppAD::AD<double> _X(const std::vector< AD<double> >& u, std::array<double, nc> c);
    CppAD::AD<double> _Z(const std::vector< AD<double> >& u, std::array<double, nc> c);
    CppAD::AD<double> _J(const std::vector< AD<double> >& u);
    std::vector< AD<double> > _u;
    //std::vector< AD<double> > _b;
    double ts, tf;
    double S, AR, cl0, cd0, e, mass;
    double rho = 1.225;


    double _tiny = 1e-6;
    const double g = 9.81;
};