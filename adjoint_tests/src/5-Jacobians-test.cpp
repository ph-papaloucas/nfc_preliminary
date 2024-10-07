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
    UAV uav(0.5, 9, 5, 0, 0);
    EngineMap engine(EngineMap::P13x65);

    //control variabls in array
    std::array<double, 2> cvars = {0.15, 30}; //theta, I

    //Variables after solving primal
    double tf = 10;
    double ts = 0;
    std::array<double, Jacobians::r> b = {1, 0, 0, 0};

    //User Input velocity
    double u, w, du;
    std::cout << "Enter velocity u: ";
    std::cin >> u;
    std::cout << "Enter velocity w: ";
    std::cin >> w;
    std::cout << "Enter du=dw for centrall diff: ";
    std::cin >> du;
    std::vector<double> xv = {0, 0, u, w} ;                // argument value for computing derivative
    std::array<double, 4> xvarr = {0, 0, u, w} ;                // argument value for computing derivative
    double thrust = engine.thrustOfWindspeedCurrent(sqrt(u*u + w*w), cvars[1]);
    std::array<double, 6> enginecoeffs = engine.getEngineCoeffs();
    std::cout << "theta = " << cvars[0] << "Current = " << cvars[1] << std::endl;

    //Prepare Jacobian object
    Jacobians j(uav, engine.getEngineCoeffs() , b, ts, tf);


    //compute Jacobian

    //std::array<std::array<double, 4>, 2> jac = j.F_u(xvarr, cvars);
    std::vector<double> jac = j.F_u(xvarr, cvars);
    std::cout << "\n";
    std::cout << "Jacobean F_u\n";
    Jacobians::printJacobian(jac, j.m, j.n);



    



    
    return 0;
}
