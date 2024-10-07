#pragma once

#include <iostream>
#include <vector>          // standard vector
#include <array>
#include <cppad/cppad.hpp>
#include <Aerodynamics.h>
#define _USE_MATH_DEFINES
#include "math.h"
#include "POly.h"
using CppAD::AD; //use AD as abbreviation for CppAD::AD

#include <UAV.h>



class Jacobians{
public:
    static const size_t r = 4; // number of param-domain space variables (b)
    static const size_t n = 4; // number of state-domain space variables
    static const size_t nc = 2; //num of conntrol variables
    static const size_t m = 2; // number of ranges space variables
    static const size_t np = 6; //number of arguments
    static const size_t nb = 4; //number of b parameters. Adjoint tries to find the dJ/db vector

    Jacobians(); //default constructor
    Jacobians(UAV uav, std::array<double, 6> p, std::array<double, r> b, double ts, double tf);

    std::vector<double> F_u(std::array<double, n> u, std::array<double, nc> c); //returns the jacobian values in a matrix
    std::vector<double> J_u(std::array<double, n> u, std::array<double, nc> c); //returns the jacobian values in a matrix
    std::vector<double> F_b(std::array<double, n> u, std::array<double, nc> c, std::array<double, nb> b, double t);
    std::vector<double> J_b(std::array<double, n> u, std::array<double, nc> c, std::array<double, nb> b);



    CppAD::ADFun<double> F_u_fun(std::array<double, nc> c); //returns the CppAD "equation" 
    CppAD::ADFun<double> J_u_fun();
    CppAD::ADFun<double> F_b_fun(std::array<double, nc> c, double t);
    CppAD::ADFun<double> J_b_fun();

    static void printJacobian(const std::vector<double>& jac, int rows, int columns);
    static void printJacobian(const std::array<std::array<double, n>, m>& jac);

    static std::array<std::array<double, n>, m> getJacMatrix(const std::vector<double>& jac);
    static std::array<std::array<double, 1>, n> getJacMatrix1(const std::vector<double>& jac);


private:
    std::array<double, r> b; 
    std::array<double, 6> p; //coefficients from engine map
    CppAD::AD<double> _getThrust(const std::vector< AD<double> >& u, std::array<double, nc> c);
    CppAD::AD<double> _J(const std::vector< AD<double> >& u);

 
    std::vector< AD<double> > _u;
    std::vector <AD<double> > _b;
    double ts, tf;
    double mass;

    double rho = 1.225;
    double _tiny = 1e-6;
    const double g = 9.81;

    Aerodynamics _aero;
    UAV _uav;
};