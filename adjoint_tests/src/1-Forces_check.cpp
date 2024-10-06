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
    std::array<double, 2> cvars = {0, 30}; //theta, I

    //Variables after solving primal
    double tf = 10;
    double ts = 0;
    std::array<double, Jacobians::r> b = {1, 0, 0, 0};

    //User Input velocity
    double u, w;
    std::cout << "Enter velocity u: ";
    //std::cin >> u;
    std::cout << "Enter velocity w: ";
    //std::cin >> w;
    u = 5;
    w = 3;
    std::vector<double> xv = {0, 0, u, w} ;                // argument value for computing derivative
    double thrust = engine.thrustOfWindspeedCurrent(sqrt(u*u + w*w), cvars[1]);

    //Prepare Jacobian object
    std::array<double, 6> enginecoeffs = engine.getEngineCoeffs();
    std::cout << "engine coeffs = " << std::endl;
    for (int i=0; i<6;++i){
        std::cout << enginecoeffs[i] << " ";
    }
    Jacobians j(uav, engine.getEngineCoeffs() , b, ts, tf);
    CppAD::ADFun<double> f;
    
    // Forces Check:
    std::cout << "\n\n\n======================CHECKINT AERO FORCES: \n\n";
    f = j.F_u_fun(cvars);
    std::vector<double> F = f.Forward(0, xv);
    std::cout << "Forces = " << F[0] << " " << F[1] << std::endl;
    Aerodynamics aero(uav);
    std::array<double, 2> Faero = aero.getAeroForcesEarthframe({xv[2], xv[3]}, cvars[0]);

    std::cout << "Faero = " << Faero[0] + thrust*cos(cvars[0])<< " " << Faero[1] - uav.getTotalMass()*9.81 + thrust*sin(cvars[0]) << std::endl;

    std::cout << "CHANGING THETA AND EVALUATE AGAIN, TO SEE IF ITS REGISTERED\n";
    cvars[0]= 0.15;
    f = j.F_u_fun(cvars);
    F = f.Forward(0, xv);
    std::cout << "F(theta = " << cvars[0] << ") = " << F[0] << " " << F[1] << std::endl;
    Faero = aero.getAeroForcesEarthframe({xv[2], xv[3]}, cvars[0]);
    std::cout << "Faero(theta = " << cvars[0] << ") = " << Faero[0] + thrust*cos(cvars[0])<< " " << Faero[1] - uav.getTotalMass()*9.81 + thrust*sin(cvars[0]) << std::endl;
    
    return 0;
}
