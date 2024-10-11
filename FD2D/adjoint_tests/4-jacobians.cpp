#include <iostream>
#include <vector>          // standard vector
#include <cppad/cppad.hpp>

#define _USE_MATH_DEFINES
#include "math.h"
using CppAD::AD; //use AD as abbreviation for CppAD::AD

#include <Jacobians.h>
#include <UAV.h>
#include <Aerodynamics.h>
#include <EngineMap.h>


int main() {
    //objects uav and enginemap
    double mass = 1;
    UAV uav(0.5, 9, mass, 0, 0);
    EngineMap engine(EngineMap::P13x65);

    //control variabls in array
    std::array<double, 2> cvars = {0.15, 30}; //theta, I

    //Variables after solving primal
    double tf = 10;
    double ts = 0;
    std::array<double, Jacobians::r> b = {1, 2,3, 4};

    //User Input velocity
    double u, w, t;
    std::cout << "Enter velocity u: ";
    std::cin >> u;
    std::cout << "Enter velocity w: ";
    std::cin >> w;
    std::cout << "Enter time to evaluate ()_b jacobians: ";
    std::cin >> t;
    std::vector<double> xv = {0, 0, u, w} ;                // argument value for computing derivative
    std::array<double, 4> xvarr = {0, 0, u, w} ;                // argument value for computing derivative
    double thrust = engine.thrustOfWindspeedCurrent(sqrt(u*u + w*w), cvars[1]);
    std::array<double, 6> enginecoeffs = engine.getEngineCoeffs();
    std::cout << "theta = " << cvars[0] << "Current = " << cvars[1] << std::endl;

    //Prepare Jacobian object
    Jacobians j(uav, engine.getEngineCoeffs() , b, ts, tf);


    //compute Jacobian

    //std::array<std::array<double, 4>, 2> jac = j.F_u(xvarr, cvars);
    std::vector<double> jacfu = j.F_u(xvarr, cvars);
    std::cout << "\n";
    std::cout << "Jacobean F_u\n";
    Jacobians::printJacobian(jacfu, j.m, j.n);


    //std::array<std::array<double, 4>, 2> jac = j.F_u(xvarr, cvars);
    std::vector<double> jacfb = j.F_b(xvarr, cvars, t);
    std::cout << "\n";
    std::cout << "Jacobean F_b\n";
    Jacobians::printJacobian(jacfb, j.m, j.r);

    //std::array<std::array<double, 4>, 4> jac = j.F_u(xvarr, cvars);
    std::vector<double> jacrb = j.R_b(xvarr, cvars, t);
    std::cout << "\n";
    std::cout << "Jacobean R_b for mass = 1 (should be = -F_b)\n";
    Jacobians::printJacobian(jacrb, j.n, j.r);


    //std::array<std::array<double, 4>, 2> jac = j.F_u(xvarr, cvars);
    std::vector<double> jacju = j.J_u(xvarr, cvars);
    std::cout << "\n";
    std::cout << "Jacobean J_u\n";
    Jacobians::printJacobian(jacju, 1, j.n);


    //std::array<std::array<double, 4>, 2> jac = j.F_u(xvarr, cvars);
    std::vector<double> jacjb = j.J_b(xvarr, cvars, t);
    std::cout << "\n";
    std::cout << "Jacobean J_b\n";
    Jacobians::printJacobian(jacjb, 1, j.r);








    

    
    return 0;
}
