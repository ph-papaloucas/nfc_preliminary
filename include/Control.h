#pragma once
#include "EngineMap.h"
#include "Aerodynamics.h"
#include "State.h"
#include "UAV.h"


class Control {
public:

    enum ThrustMode{
        THRUST_UNDEFINED,
        MAX
    };

    enum ControlMode {
       UNDEFINED,
        TAKEOFF,
        TRIM,
    };

    Control();
    Control(EngineMap engine, const UAV& uav, double max_amps);
    void setControlMode(ControlMode control_mode){_current_control_mode = control_mode;};

    // void applyBoundaries(std::array<double, 2> &forces, double altitude, double mass){
    //     std::cout << "force[1] = " << forces[1] << ".... weight force = " << mass*9.81 << std::endl;
    //     if ( (_current_control_mode == TAKEOFF) && (altitude < 0.001) ){
    //         if (forces[1] < mass*9.81)
    //             forces[1] = 0e0;
    //     }

    // }



    // std::array<double, 2> getForces(const State &state, ControlState& control_state);
    // bool checkTermination(double Fz){
    //     // case (_current_control_mode){
    //     // switch TAKEOFF:
    //     //     if 

    //     //     break;

    //     // }


    //     return false;
    // }


private:
    double _getGamma(std::array<double, 2> velocity);
    double _getThrust(std::array<double, 2> bodyframe_velocity);

    void _applyControl(ControlState &control_state, std::array<double,2 > velocity);
    void _applyBoundaries(std::array<double, 2> &forces, double altitude);
    void _applyGroundEffect(std::array<double, 2> &forces);
    
    double _max_amps;
    EngineMap _engine;
    ControlMode _current_control_mode;
    ThrustMode _current_thrust_mode;
    Aerodynamics _aero;
    UAV _uav;
};