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

    Control(const UAV& uav, const EngineMap& engine, double max_amps);

    void setControlMode(ControlMode control_mode, std::array<double, 4> coeffs);     //set control for stuff that need polynomyal
    void setControlMode(ControlMode control_mode);      //set control for stuff that dont need polynomial

    //void setControlMode(ControlMode control_mode){_current_control_mode = control_mode;};
    void setTerminationMode(TerminationMode term_mode){_termination_mode = term_mode;};

    std::array<double, 2> getForces(const State &state, ControlState& control_state, double t);
    
    bool checkTermination(const State &state, std::array<double, 2> u);
    void applyBoundaries(std::array<double, 2> &forces, double altitude);

    template <typename T>
    T getThrust(std::array<T, 2> velocity_bodyframe) const{
        T vel_wind = velocity_bodyframe[0];
        T thrust = 0 ;

        switch (_current_thrust_mode){
            case MAX:
                thrust =  _engine.thrustOfWindspeedCurrent(vel_wind, _max_amps);
                break;
            case THRUST_UNDEFINED:
                std::cerr << "Error: Undefined thrust mode!" << std::endl;
                std::exit(EXIT_FAILURE);
                break;
            default:
                std::cerr << "Error: Thrust mode unknown value!" << std::endl;
                std::exit(EXIT_FAILURE);
                break;
        }
        if (thrust<0){
            thrust = 0;
        }
        return thrust;
    }


private:
    bool apply_ground_effect = false; //CHANGE ME
    double _getTheta(std::array<double, 2> velocity, double t, bool apply_ground_effect, double height);

    void _applyControl(ControlState &control_state, std::array<double,2 > velocity, double t, bool apply_ground_effect, double height);

    
    double _max_amps;
    const double _max_theta = Aerodynamics::deg2rad(90);

    ControlMode _current_control_mode;
    ThrustMode _current_thrust_mode;
    TerminationMode _termination_mode;
    Aerodynamics _aero;
    const UAV& _uav;
    const EngineMap& _engine;
    static const size_t _n_coeffs = 4;
    Poly<double, _n_coeffs> _poly;
    double _ceil = 100;

    friend class StateSpace;
    friend class AdjointControl;
    friend class AdjointStateSpace;
};