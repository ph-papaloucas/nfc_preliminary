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

    double getThetaForTrim(std::array<double, 2> velocity, double thrust);


    //std::array<double, 2> getForcesBodyframe(std::array<double, 2> velocity_bodyframe);
    std::array<double, 2> getAeroForcesEarthframe(std::array<double, 2> velocity, double  theta, bool apply_ground_effect, double height, double wingspan, double AR);
    std::array<double, 2> getAeroForcesEarthframe(std::array<double, 2> velocity, double  theta);


    static std::array<double, 2> rotateFromEarth2Bodyframe(std::array<double, 2> vector, double theta);
    static std::array<double,2 > rotateFromWind2Earthframe(std::array<double, 2> vector, std::array<double, 2> velocity_vector);
    static std::array<double, 2> rotateFromBody2Earthframe(std::array<double, 2> vector, double theta);
    static double rad2deg(double rad);
    static double deg2rad(double deg);
private:
    double _e = 0.8; //oswald coeff
    double _qinf(double velocity);
    UAV _uav;
};