#pragma once
#include "Airplane.h"
#include "State.h"

#include <cmath>

class Aerodynamics{
public:
    Aerodynamics(const Airplane& plane);

    State getCruiseState(double velocity);


    static double rad2deg(double rad);
    static double deg2rad(double deg);
private:
    double _qinf(double velocity);
    Airplane _plane;
};