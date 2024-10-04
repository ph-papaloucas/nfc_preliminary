#pragma once
#include "EngineMap.h"
#include "Aerodynamics.h"
#include "State.h"
#include "UAV.h"
#include <tuple>

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
        TRIM,
    };

    Control();
    Control(EngineMap engine, const UAV& uav, double max_amps);
    void setControlMode(ControlMode control_mode){_current_control_mode = control_mode;};

    std::array<double, 2> getForces(const State &state, ControlState& control_state);
    
    bool checkTermination(const State &state, std::array<double, 2> u);
    void applyBoundaries(std::array<double, 2> &forces, double altitude);


    double getThrust(std::array<double, 2> bodyframe_velocity) const;
private:
    double _getTheta(std::array<double, 2> velocity);

    void _applyControl(ControlState &control_state, std::array<double,2 > velocity);

    void _applyGroundEffect(std::array<double, 2> &forces);
    
    double _max_amps;
    EngineMap _engine;
    ControlMode _current_control_mode;
    ThrustMode _current_thrust_mode;
    Aerodynamics _aero;
    UAV _uav;
};