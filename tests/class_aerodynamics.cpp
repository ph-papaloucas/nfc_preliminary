#include "UAV.h"
#include "Aerodynamics.h"

#include <iostream>

int main(){
    std::cout <<"Test executable for class Airplane\n";
    UAV vtol(0.5, 9, 10, 0.01, 1);

    
    Aerodynamics aero(vtol);
    std::cout << aero.getCruiseState(20);


    std::cout << "======Test Rotations to bodyframe======\n";
    std::array<double, 2> v = {1, 0};
    std::cout << "Rotating {" << v[0] << ", " << v[1] << "} from earth frame to bodyframe, gamma = 30deg\n";
    std::array<double, 2> vtemp =  Aerodynamics::rotateVector2Bodyframe({1,0}, Aerodynamics::deg2rad(30));
    std::cout << "=> {"<< vtemp[0] << ", " << vtemp[1] << "}\n";
    std::cout << "Rotating {" << vtemp[0] << ", " << vtemp[1] << "} back to earth frame\n" ;
    vtemp =  Aerodynamics::rotateVector2Earthframe(vtemp, Aerodynamics::deg2rad(30));
    std::cout << "=> {"<< vtemp[0] << ", " << vtemp[1] << "}\n";



    std::cout << "======Test Aeroforces bodyframe======\n";
    std::array<double, 2> u;
    std::array<double, 2> vel = {10,10};
    u = aero.getForcesBodyframe(vel);
    std::cout << " velocity = " << vel[0] << " " << vel[1] << "Forces = " << u[0] << " " << u[1] << std::endl; 


                    

    return 0;
}