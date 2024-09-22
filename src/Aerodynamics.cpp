#include "Aerodynamics.h"

Aerodynamics::Aerodynamics(const UAV& uav):_uav(uav){};

double Aerodynamics::_qinf(double velocity){
    return 0.5*1.225*pow(velocity,2);
}

AeroState Aerodynamics::getCruiseState(double velocity){
    double lift = _uav.getTotalMass()*9.81;

    double cl = lift/(_qinf(velocity)*_uav.getSurface());
    double aoa = (cl - _uav.getCl0())/(2*M_PI);
    

    return AeroState(aoa, cl, velocity);
}

std::array<double, 2> Aerodynamics::rotateVector2Bodyframe(std::array<double, 2> vector, double gamma){
    return {cos(gamma)*vector[0] - sin(gamma)*vector[1],
            sin(gamma)*vector[0] + cos(gamma)*vector[1]};

}
std::array<double,2 > Aerodynamics::rotateVector2Earthframe(std::array<double, 2> vector, double gamma){
    return {cos(-gamma)*vector[0] - sin(-gamma)*vector[1],
            sin(-gamma)*vector[0] + cos(-gamma)*vector[1]};
}

double Aerodynamics::rad2deg(double rad){
    return rad*180/M_PI;
}

double Aerodynamics::deg2rad(double deg){
    return deg*M_PI/180;
}

std::array<double, 2> Aerodynamics::getForcesBodyframe(std::array<double, 2> velocity_bodyframe){
    double velocity_norm = sqrt( pow(velocity_bodyframe[0],2) + pow(velocity_bodyframe[1],2));
    double qinf = _qinf(velocity_norm);
    double aoa = atan2(velocity_bodyframe[1], velocity_bodyframe[0]);

    double cl = 2*M_PI*aoa - _uav.getCl0();
    double cd = _uav.getCd0() +pow(cl,2)/(2*M_PI*_uav.getAR()*_e);

    double lift = qinf*_uav.getSurface()*cl;
    double drag = qinf*_uav.getSurface()*cd;

    std::cout << "drag = " << drag << "  lift = " << lift <<std::endl;
    std::array<double, 2> F = {-drag, lift};

	return F;

}

double Aerodynamics::getGammaForTrim(std::array<double, 2> velocity){
    std::cerr << "SOS didnt implement Aerodynamics::getGammaForTrim() yet...\n";
    std::cerr << "SOS didnt implement Aerodynamics::getGammaForTrim() yet...\n";

    return 0;
}