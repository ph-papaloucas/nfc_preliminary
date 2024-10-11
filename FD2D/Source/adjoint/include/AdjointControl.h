#pragma once

#include <iostream>
#include <vector>          // standard vector
#include <array>
#include <cppad/cppad.hpp>
#include <Aerodynamics.h>
#define _USE_MATH_DEFINES
#include "math.h"
#include "Poly.h"
#include "Control.h"
using CppAD::AD; //use AD as abbreviation for CppAD::AD

#include <UAV.h>

class AdjointControl{
public:
    static const size_t r = 4; // number of param-domain space variables (b)
    static const size_t n = 4; // number of state-domain space variables
    static const size_t nc = 2; //num of conntrol variables that are const
    static const size_t m = 2; // number of ranges space variables
    static const size_t np = 6; //number of arguments

    AdjointControl(Control control, std::array<double, r> b, double ts, double tf);


    CppAD::ADFun<double> F_u_fun(std::array<double, nc> c); //returns the CppAD "equation" 
    CppAD::ADFun<double> F_b_fun(std::array<double, n> u, std::array<double, nc> c, double t); //returns the CppAD "equation" 
    CppAD::ADFun<double> J_u_fun();
    CppAD::ADFun<double> J_b_fun(std::array<double, n> u);
    //std::array<std::array<double, n>, m>
    std::vector<double> F_u(std::array<double, n> u, std::array<double, nc> c); //returns the jacobian values in a matrix
    //std::array<std::array<double, 1>, n>
    std::vector<double> J_u(std::array<double, n> u, std::array<double, nc> c); //returns the jacobian values in a matrix

    //std::array<std::array<double, e>, m>
    std::vector<double> F_b(std::array<double, n> u, std::array<double, nc> c, double t); //returns the jacobian values in a matrix
    std::vector<double> R_b(std::array<double, n> u, std::array<double, nc> c, double t); //returns the jacobian values in a matrix
    std::vector<double> J_b(std::array<double, n> u, std::array<double, nc> c, double t); //returns the jacobian values in a matrix

    static void printJacobian(const std::vector<double>& jac, int rows, int columns);
    static void printJacobian(const std::array<std::array<double, n>, m>& jac);

    static std::array<std::array<double, n>, m> getJacMatrix(const std::vector<double>& jac);
    static std::array<std::array<double, 1>, n> getJacMatrix1(const std::vector<double>& jac);


    template <size_t T>
    static std::vector<double> multiplyJacWithVect(std::vector<double> jac, std::array<double, T> vect){
        size_t cols = vect.size(); //cols of jac
        size_t rows= jac.size()/cols; //rows of jac

        if (jac.size() % rows != 0) {
            throw std::invalid_argument("Jacobian size is not compatible with the vector size.");
        }


        std::vector<double> result(rows, 0.0); // Result vector initialized with zeros

        // Matrix-vector multiplication
        for (size_t i = 0; i < rows; ++i) {
            for (size_t j = 0; j < cols; ++j) {
                result[i] += jac[i * cols + j] * vect[j]; // Accessing jacobian as a flat array
            }
        }

        return result;
    }

    template <size_t T>
    static std::vector<double> multiplyVectWithJac(std::array<double, T> vect, std::vector<double> jac) {
        size_t cols = jac.size() / vect.size(); // Number of columns of jac
        size_t rows = vect.size();              // Number of rows of jac (same as vector size)

        if (jac.size() % rows != 0) {
            throw std::invalid_argument("Jacobian size is not compatible with the vector size.");
        }

        std::vector<double> result(cols, 0.0); // Result vector initialized with zeros (size = number of columns)

        // Vector-matrix multiplication
        for (size_t j = 0; j < cols; ++j) {
            for (size_t i = 0; i < rows; ++i) {
                result[j] += vect[i] * jac[i * cols + j]; // Accessing jacobian as a flat array
            }
        }

    return result;
}

    double evaluateLossFunction(std::array<double, n> u){
        std::vector<AD<double>> uv = {u[0], u[1], u[2], u[3]};
        AD<double> value = _J(uv);
        return CppAD::Value(value);
    }
private:
    std::array<double, r> b; 
    std::array<double, 6> p; //coefficients from engine map
    // CppAD::AD<double> _X(const std::vector< AD<double> >& u, std::array<double, nc> c);
    CppAD::AD<double> _getThrust(const std::vector< AD<double> >& u, std::array<AD<double>, nc> c);
    // CppAD::AD<double> _Z(const std::vector< AD<double> >& u, std::array<double, nc> c);
    CppAD::AD<double> _J(const std::vector< AD<double> >& u);

    CppAD::ADFun<double> R_b_fun(std::array<double, n> u, std::array<double, nc> c, double t){
        CppAD::Independent(_b);              //declare independent variables and start recording operation sequence


        Poly <AD<double>, r> theta_poly({_b[0], _b[1], _b[2], _b[3]});
        AD<double> theta = theta_poly.getValue(t);

        std::array<AD<double>, nc> control_vector = {theta, c[1]};

        std::array<AD<double>, 2> velocity = {u[2], u[3]};
        AD<double> height = u[1] + _control._uav.getWheelOffset();
        std::vector<AD<double>> F = _aero.getAeroForcesEarthframeVector(_control._uav, velocity, control_vector[0], _control.apply_ground_effect, height);

        AD<double> thrust = _getThrust(_u, control_vector);
        F[0] = F[0]  + thrust*cos(c[0]);  
        F[1] = F[1] - _control._uav.getTotalMass()*9.81 + thrust*sin(c[0]);

    
        std::vector< AD<double> > R(n);
        R[0] = - _u[2];
        R[1] = - _u[3];
        R[2] = - (1/_control._uav.getTotalMass())*F[0];
        R[3] = - (1/_control._uav.getTotalMass())*F[1];

        CppAD::ADFun<double> Rfun(_b, R);   // store operation sequence in f: X -> Y and stop recording (x = states/independent vars -> Y = outputs)
        return Rfun;
    }

 
    CppAD::ADFun<double> _F_b();
    CppAD::ADFun<double> _J_b();
    std::vector< AD<double> > _u;
    std::vector< AD<double> > _b;
    double ts, tf;
    //double S, AR, cl0, cd0, e, mass;
    double rho = 1.225;


    double _tiny = 1e-6;
    const double g = 9.81;
    Control _control;
    Aerodynamics _aero;

    friend class AdjointStateSpace;

};