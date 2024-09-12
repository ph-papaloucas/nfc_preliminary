#pragma once

#include <iostream>
#include <cmath>

class State{
public:
    State(double angle_of_attack, double cl, double velocity);

    double angle_of_attack, cl, velocity;

    //overload std::cout function
    friend std::ostream& operator<<(std::ostream& os, const State& state) {
    os << "angle_of_attack [deg]= " << state.angle_of_attack*180/M_PI
        << ", cl = " << state.cl
        << ", velocity = " << state.velocity;
    return os;
    }
};