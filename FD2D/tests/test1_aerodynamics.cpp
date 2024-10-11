#include <string>

#include "UAV.h"
#include "Aerodynamics.h"
#include "Control.h"
#include "EngineMap.h"
void printStuff(double theta, std::array<double, 2> velocity){
    std::cout << "\n==================\n theta = "  << Aerodynamics::rad2deg(theta) << "[deg]. velocity = " << velocity[0] << " " << velocity[1] << std::endl;
}

void printForce(std::array<double, 2> F){
    std::cout << "Fx = " << F[0] << ". Fz = " <<F[1] << std::endl;
}


int main(){
    double S = 0.5;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(-4);
    double mass = 4;
    UAV p1(S, AR, mass, a0, 1);
    Aerodynamics aero(p1);
    std::cout << " Plane that im solving for :\n";
    std::cout << "S    AR    a0 = " << S << "    " << AR << "    " << a0 << std::endl;
    std::array<double,2> F = {0,0};
    double theta = 0;
    std::array<double, 2> velocity = {0,0};
    double a = 0;




    std::cout << "TESTING VELOCITY: \n";
    std::cout << "Check aoa resulting from wind angle \n";
    velocity = {-1, 0};
    std::cout << "< velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    a = aero.getAoa1(velocity);
std::cout << "a = " << Aerodynamics::deg2rad(a) << "[deg]\n";
    std::cout << std::endl;
    velocity = {0, 1};
    std::cout << "^ velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    a = aero.getAoa1(velocity);
std::cout << "a = " << Aerodynamics::deg2rad(a) << "[deg]\n";
        std::cout << std::endl;
    velocity = {1, 0};
    std::cout << "> velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    a = aero.getAoa1(velocity);
std::cout << "a = " << Aerodynamics::deg2rad(a) << "[deg]\n";
        std::cout << std::endl;
    velocity = {0, -1};
    std::cout << "velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    a = aero.getAoa1(velocity);
std::cout << "a = " << Aerodynamics::deg2rad(a) << "[deg]\n";
        std::cout << std::endl;

    velocity = {-1, -1};
    std::cout << "< velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    a = aero.getAoa1(velocity);
std::cout << "a = " << Aerodynamics::deg2rad(a) << "[deg]\n";
        std::cout << std::endl;
    velocity = {-1, 1};
    std::cout << "^ velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    a = aero.getAoa1(velocity);
std::cout << "a = " << Aerodynamics::deg2rad(a) << "[deg]\n";
        std::cout << std::endl;
    velocity = {1, 1};
    std::cout << "> velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    a = aero.getAoa1(velocity);
std::cout << "a = " << Aerodynamics::deg2rad(a) << "[deg]\n";
        std::cout << std::endl;
    velocity = {1, -1};
    std::cout << "velocity = " << velocity[0] << " " << velocity[1] << std::endl;
    a = aero.getAoa1(velocity);
std::cout << "a = " << Aerodynamics::deg2rad(a) << "[deg]\n";
        std::cout << std::endl;



    std::cout << "\n\n\n========================\nTesting theta\n";

    theta = 0;
    velocity = {1,0};
    printStuff(theta, velocity);
    F = aero.getAeroForcesEarthframe(velocity, theta);
    printForce(F);

    theta = Aerodynamics::deg2rad(10);
    velocity = {10, 0};
    printStuff(theta, velocity);
    F = aero.getAeroForcesEarthframe(velocity, theta);
    printForce(F);

    theta = Aerodynamics::deg2rad(-10);
    velocity = {10, 0};
    printStuff(theta, velocity);
    F = aero.getAeroForcesEarthframe(velocity, theta);
    printForce(F);

    std::cout << "\n\n\n========================\nTesting thetaForTrim()\n\n";
    double thrust = 0;
    EngineMap engine(EngineMap::P13x65);
    Control control(engine, p1, 10);
    double velocity_norm = 15;


    velocity = {velocity_norm  ,0};
    theta = aero.getThetaForTrim(velocity, control, false, 0e0);
    printStuff(theta, velocity);
    std::cout << "aoa = " << Aerodynamics::rad2deg(Aerodynamics::getAoa1(velocity)) << std::endl;
    F = aero.getAeroForcesEarthframe(velocity, theta);
    printForce(F);
    thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, theta));
    std::cout << " Fztotal = " << F[1] - p1.getTotalMass()*9.81 + thrust*sin(theta) << std::endl;


    velocity = {sqrt(pow(velocity_norm, 2)/2),sqrt(pow(velocity_norm, 2)/2)};
    theta = aero.getThetaForTrim(velocity, control, false, 0e0);
    printStuff(theta, velocity);
    std::cout << "aoa = " << Aerodynamics::rad2deg(Aerodynamics::getAoa1(velocity)) << std::endl;
    F = aero.getAeroForcesEarthframe(velocity, theta);
    printForce(F);
    thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, theta));
    std::cout << " Fztotal = " << F[1] - p1.getTotalMass()*9.81 + thrust*sin(theta) << std::endl;

    velocity = {sqrt(pow(velocity_norm, 2)/2),-sqrt(pow(velocity_norm, 2)/2)};
    theta = aero.getThetaForTrim(velocity, control, false, 0e0);
    printStuff(theta, velocity);
    std::cout << "aoa = " << Aerodynamics::rad2deg(Aerodynamics::getAoa1(velocity)) << std::endl;
    F = aero.getAeroForcesEarthframe(velocity, theta);
    printForce(F);
    thrust = control.getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, theta));
    std::cout << " Fztotal = " << F[1] - p1.getTotalMass()*9.81 + thrust*sin(theta) << std::endl;



    return 0 ;
}
