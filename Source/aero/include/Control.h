#pragma once
#include "EngineMap.h"
#include "Aerodynamics.h"
#include "State.h"
#include "UAV.h"
#include "Poly.h"
#include <stdexcept>
#include <tuple>

#include <cppad/cppad.hpp>
using CppAD::AD; 

class Control {
public:

    enum ThrustMode{
        THRUST_UNDEFINED,
        MAX
    };

    enum ControlMode {
       UNDEFINED,
        TAKEOFF,
        TRIM_GAMMA,
        THETA,
    };

    enum TerminationMode{
        REACH_CEIL
    };

    Control();
    Control(EngineMap engine, const UAV& uav, double max_amps);
    
    //set control for stuff that need polynomyal
    void setControlMode(ControlMode control_mode, std::array<double, 4> coeffs);
    //set control for stuff that dont need polynomial
    void setControlMode(ControlMode control_mode);

    //void setControlMode(ControlMode control_mode){_current_control_mode = control_mode;};
    void setTerminationMode(TerminationMode term_mode){_termination_mode = term_mode;};

    std::array<double, 2> getForces(const State &state, ControlState& control_state, double t);
    
    bool checkTermination(const State &state, std::array<double, 2> u);
    void applyBoundaries(std::array<double, 2> &forces, double altitude);


    double getThrust(std::array<double, 2> bodyframe_velocity) const;
    CppAD::AD<double> getThrust(std::array<CppAD::AD<double>, 2> bodyframe_velocity) const;
private:
    double _getTheta(std::array<double, 2> velocity, double t, bool apply_ground_effect, double height);

    void _applyControl(ControlState &control_state, std::array<double,2 > velocity, double t, bool apply_ground_effect, double height);

    
    double _max_amps;
    EngineMap _engine;
    ControlMode _current_control_mode;
    ThrustMode _current_thrust_mode;
    TerminationMode _termination_mode;
    Aerodynamics _aero;
    UAV _uav;
    static const size_t _n_coeffs = 4;
    Poly<_n_coeffs> _poly;
    double _ceil = 100;
};