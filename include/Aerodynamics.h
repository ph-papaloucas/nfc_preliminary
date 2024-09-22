#pragma once
#include "UAV.h"
#include "State.h"
#include <array>

#include <cmath>

class Aerodynamics{
public:
    Aerodynamics(){};
    Aerodynamics(const UAV& uav);

    AeroState getCruiseState(double velocity);

    double getGammaForTrim(std::array<double, 2> velocity);

    std::array<double, 2> getForcesBodyframe(std::array<double, 2> velocity_bodyframe);


    static std::array<double, 2> rotateVector2Bodyframe(std::array<double, 2> vector, double gamma);
    static std::array<double,2 > rotateVector2Earthframe(std::array<double, 2> vector, double gamma);
    static double rad2deg(double rad);
    static double deg2rad(double deg);
private:
    double _e = 0.8; //oswald coeff
    double _qinf(double velocity);
    UAV _uav;
};