#pragma once

#include <vector>
#include <iostream>
#include <iomanip>
#include <array>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>

class AeroState{
public:
    AeroState(double angle_of_attack, double cl, double velocity);

    double angle_of_attack, cl, velocity;

    //overload std::cout function
    friend std::ostream& operator<<(std::ostream& os, const AeroState& state) {
    os << "angle_of_attack [deg]= " << state.angle_of_attack*180/M_PI
        << ", cl = " << state.cl
        << ", velocity = " << state.velocity;
    return os;
    }
};

class ControlState{
public:
    ControlState();
    ControlState(double theta, double thrust);

    double theta;
    double thrust;
};

class State{
public:
    State();
    State(std::array<double, 4> x, std::array<double, 2> u);
    State(double x, double z, double u, double w, double Fx, double Fz);


    void setState(double x, double z, double u, double w);
    void setForces(double Fx, double Fz);

    std::array<double,4 > getStatesArray();
    std::array<double,2 > getForcesArray();

    double altitude(){
        return z;
    }

    friend std::ostream& operator<<(std::ostream& out, const State& data){
        out << std::fixed << std::setprecision(6); // Set fixed-point notation and precision to 2 decimal places
        out << "x        z        u        w        theta    Fx       Fz\n"; 
        out     << data.x << ' ' 
                << data.z << ' ' 
                << data.u << ' ' 
                << data.w << ' '
                << data.Fx << ' '
                << data.Fz << "\n";
    
        return out;
    }


    double x, z, u, w;
    double Fx, Fz;
};


class StateHistory{
public:
    StateHistory();

    void prepareHistoryMatrix(int total_timesteps, double t0, const State& initial_state, const ControlState& control_state);
    void appendState(double t0, const State& state, const ControlState& control_state);


    friend std::ostream& operator<<(std::ostream& out, const StateHistory& data);
    void printToCsv(const std::string& filename);

    double getLastTime(){
        return _time.back();
    }

private:
    std::vector<State> _history;
    std::vector<ControlState> _control_history;
    std::vector<double> _time;
};