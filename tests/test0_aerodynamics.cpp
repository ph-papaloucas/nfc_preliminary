#include <string>

#include "UAV.h"
#include "Aerodynamics.h"

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


    double gamma;
    std::stringstream(gamma_str) >> gamma;
    gamma = Aerodynamics::deg2rad(gamma);


    std::array<double, 2> velocity = {10, 0 };
    std::cout << "Velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    std::cout << "S    AR    a0 = " << S << "    " << AR << "    " << a0 << std::endl;
    std::array<double,2> forces_bodyframe = aero.getForcesBodyframe(Aerodynamics::rotateVector2Bodyframe(velocity, gamma));
    std::array<double, 2> F  = Aerodynamics::rotateVector2Earthframe(forces_bodyframe, gamma);
    std::cout << "Lift = " << forces_bodyframe[1] << "    Drag = " << -forces_bodyframe[0] << std::endl;
    std::cout << "Forces earthaxis = " << F[0] << " " << F[1] << std::endl;

    return 0 ;
}