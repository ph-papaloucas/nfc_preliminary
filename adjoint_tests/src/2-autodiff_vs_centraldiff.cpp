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
    double thrust = engine.thrustOfWindspeedCurrent(sqrt(u*u + w*w), cvars[1]);
    std::array<double, 6> enginecoeffs = engine.getEngineCoeffs();
    std::cout << "theta = " << cvars[0] << "Current = " << cvars[1] << std::endl;

    //Prepare Jacobian object
    Jacobians j(uav, engine.getEngineCoeffs() , b, ts, tf);


    //compute Jacobian
    CppAD::ADFun<double> f;
    f = j.F_u(cvars);
    std::vector<double> jac(j.m * j.n); // Jacobian of f (m by n matrix)


    jac  = f.Jacobian(xv);      // Jacobian for operation sequence
    std::cout << "\n";
    std::cout << "Jacobean F_u\n";
    Jacobians::printJacobian(jac, j.m, j.n);

    // Central, frond and rear differences in respect to ux
    std::vector<double> jac_central(j.m * j.n);
    std::vector<double> jac_front(j.m * j.n);
    std::vector<double> jac_rear(j.m * j.n);

    
    std::vector<double> xv_temp;
    std::vector<double> F = f.Forward(0, xv);
    std::vector<double> Ffront(2);
    std::vector<double> Frear(2);

    int jindex;

    
    //x
    jindex = 0;
    xv_temp = {xv[0] + du, xv[1], xv[2], xv[3]};
    Ffront = f.Forward(0, xv_temp);
    xv_temp = {xv[0] - du, xv[1], xv[2], xv[3]};
    Frear = f.Forward(0, xv_temp);

    jac_central[jindex] = (Ffront[0] - Frear[0])/(2*du);
    jac_central[j.m*jindex + jindex] = (Ffront[1] - Frear[1])/(2*du);
    jac_front[jindex] = (Ffront[0] - F[0])/(du);
    jac_front[j.m*jindex + jindex] = (Ffront[1] - F[1])/(du);
    jac_rear[jindex] = (F[0] - Frear[0])/(du);
    jac_rear[j.m*jindex + jindex] = (F[1] - Frear[1])/(du);


    double iz = 1; // index for 2nd equation Z
    //z
    jindex = 1;
    xv_temp = {xv[0], xv[1] + du, xv[2], xv[3]};
    Ffront = f.Forward(0, xv_temp);
    xv_temp = {xv[0], xv[1] - du, xv[2], xv[3]};
    Frear = f.Forward(0, xv_temp);

    jac_central[jindex] = (Ffront[0] - Frear[0])/(2*du);
    jac_central[iz*j.n + jindex] = (Ffront[1] - Frear[1])/(2*du);
    jac_front[jindex] = (Ffront[0] - F[0])/(du);
    jac_front[iz*j.n  + jindex] = (Ffront[1] - F[1])/(du);
    jac_rear[jindex] = (F[0] - Frear[0])/(du);
    jac_rear[iz*j.n  + jindex] = (F[1] - Frear[1])/(du);

    //ux
    jindex = 2;
    xv_temp = {xv[0], xv[1], xv[2] + du, xv[3]};
    Ffront = f.Forward(0, xv_temp);
    xv_temp = {xv[0], xv[1], xv[2] - du, xv[3]};
    Frear = f.Forward(0, xv_temp);

    jac_central[jindex] = (Ffront[0] - Frear[0])/(2*du);
    jac_central[iz*j.n + jindex] = (Ffront[1] - Frear[1])/(2*du);
    jac_front[jindex] = (Ffront[0] - F[0])/(du);
    jac_front[iz*j.n  + jindex] = (Ffront[1] - F[1])/(du);
    jac_rear[jindex] = (F[0] - Frear[0])/(du);
    jac_rear[iz*j.n  + jindex] = (F[1] - Frear[1])/(du);

    //uz = w
    jindex = 3;
    xv_temp = {xv[0], xv[1], xv[2], xv[3] + du};
    Ffront = f.Forward(0, xv_temp);
    xv_temp = {xv[0], xv[1], xv[2], xv[3] - du};
    Frear = f.Forward(0, xv_temp);

    jac_central[jindex] = (Ffront[0] - Frear[0])/(2*du);
    jac_central[iz*j.n + jindex] = (Ffront[1] - Frear[1])/(2*du);
    jac_front[jindex] = (Ffront[0] - F[0])/(du);
    jac_front[iz*j.n  + jindex] = (Ffront[1] - F[1])/(du);
    jac_rear[jindex] = (F[0] - Frear[0])/(du);
    jac_rear[iz*j.n  + jindex] = (F[1] - Frear[1])/(du);

    std::cout << "\n";
    std::cout << "Jacobean F_u central\n";
    Jacobians::printJacobian(jac_central, j.m, j.n);
    std::cout << "\n";
    std::cout << "Jacobean F_u front\n";
    Jacobians::printJacobian(jac_front, j.m, j.n);
    std::cout << "\n";
    std::cout << "Jacobean F_u rear\n";
    Jacobians::printJacobian(jac_rear, j.m, j.n);


    

    
    return 0;
}
