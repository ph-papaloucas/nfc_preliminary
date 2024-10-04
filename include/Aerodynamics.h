#pragma once
#include "UAV.h"
#include "State.h"
#include <array>

#include <cmath>


class Control; //forward declaration
class Aerodynamics{
public:
    Aerodynamics(){};
    Aerodynamics(const UAV& uav);

    AeroState getCruiseState(double velocity);

    double getThetaForTrim(std::array<double, 2> velocity,const Control& control);


    //std::array<double, 2> getForcesBodyframe(std::array<double, 2> velocity_bodyframe);
    std::array<double, 2> getAeroForcesEarthframe(std::array<double, 2> velocity, double theta,  bool apply_ground_effect, double height);
    std::array<double, 2> getAeroForcesEarthframe(std::array<double, 2> velocity, double  theta);
    std::array<double, 2> getCoeffs(std::array<double, 2> velocity, double theta, bool apply_ground_effect, double height);


    static std::array<double, 2> rotateFromEarth2Bodyframe(std::array<double, 2> vector, double theta);
    static std::array<double,2 > rotateFromWind2Earthframe(std::array<double, 2> vector, std::array<double, 2> velocity_vector);
    static std::array<double, 2> rotateFromBody2Earthframe(std::array<double, 2> vector, double theta);
    static double rad2deg(double rad);
    static double deg2rad(double deg);
    static double _getAoa1(std::array<double, 2> velocity);//this is without theta
private:
    double _qinf(double velocity);
    UAV _uav;
};