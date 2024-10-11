#include <string>

#include "UAV.h"
#include "Aerodynamics.h"
#include <stdlib.h>

int main(){
    double S = 0.5;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(-4);
    double mass = 4;
    UAV p1(S, AR, mass, a0, 1);
    Aerodynamics aero(p1);


    std::cout << "Give angle gamma in degres:" ;
    std::string gamma_str;
    std::cin >> gamma_str;


    double gamma = atof(gamma_str.c_str());
    gamma = Aerodynamics::deg2rad(gamma);


    std::array<double, 2> velocity = {10, -2 };
    std::cout << "Velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    std::cout << "S    AR    a0 = " << S << "    " << AR << "    " << a0 << std::endl;
    std::array<double,2> F = aero.getAeroForcesEarthframe(velocity, gamma);
    std::cout << "F = " << F[0] << " " << F[1] << std::endl;

    return 0 ;
}