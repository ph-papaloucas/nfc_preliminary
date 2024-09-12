#include "Aerodynamics.h"

Aerodynamics::Aerodynamics(const Airplane& plane):_plane(plane){};

double Aerodynamics::_qinf(double velocity){
    return 0.5*1.225*pow(velocity,2);
}

State Aerodynamics::getCruiseState(double velocity){
    double lift = _plane.getTotalMass()*9.81;

    double cl = lift/(_qinf(velocity)*_plane.getSurface());
    double aoa = (cl - _plane.getCl0())/(2*M_PI);
    

    return State(aoa, cl, velocity);
}

double Aerodynamics::rad2deg(double rad){
    return rad*180/M_PI;
}

double Aerodynamics::deg2rad(double deg){
    return deg*M_PI/180;
}