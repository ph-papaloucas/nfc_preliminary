#include "Control.h"

Control::Control():_max_amps(0), _current_thrust_mode(MAX){};
Control::Control(EngineMap engine, const UAV& uav, double max_amps):_engine(engine),_uav(uav), _aero(uav), _max_amps(max_amps), _current_thrust_mode(MAX){};

    //set control for stuff that need polynomyal
void Control::setControlMode(ControlMode control_mode, std::array<double, 4> coeffs){
    if (control_mode == THETA){
        _current_control_mode = control_mode;
        _poly = Poly<double, _n_coeffs>(coeffs);
    }
    else{
        throw std::invalid_argument("Invalid control mode set. Use the appropriate setControlMode method which does NOT ask for a polynomial");
    }
};
//set control for stuff that dont need polynomial
void Control::setControlMode(ControlMode control_mode){
    if (control_mode != THETA)
        _current_control_mode = control_mode;
    else{
        throw std::invalid_argument("Invalid control mode set. Use the appropriate setControlMode method which asks for a polynomial");
    }
}


double Control::_getTheta(std::array<double, 2> velocity, double t, bool apply_ground_effect, double height){
    double theta = 0;
    #ifdef DEBUG
        if(VERBOSITY_LEVEL>=3){
            std::cout << "CONTROL MODE = " << _current_control_mode << std::endl;
        }
    #endif
    switch (_current_control_mode){
        case TAKEOFF:
            theta =  _uav.getAoaTakeoff();
            break;
        case TRIM_GAMMA:
            theta = _aero.getThetaForTrim(velocity, *(this), apply_ground_effect, height);
            break;
        case THETA:
            theta = _poly.getValue(t);
            break;

    }
    return theta;
}

void Control::_applyControl(ControlState &control_state, std::array<double,2 > velocity, double t, bool apply_ground_effect, double height){
    control_state.theta = _getTheta(velocity, t, apply_ground_effect, height);
    control_state.thrust = getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, control_state.theta));


}

std::array<double, 2> Control::getForces(const State &state, ControlState& control_state, double t){
    std::array<double, 2> velocity = {state.u, state.w};
    bool apply_ground_effect = true; //CHANGE ME
    _applyControl(control_state, velocity, t, apply_ground_effect, state.z + _uav.getWheelOffset());

    std::array<double, 2> u = _aero.getAeroForcesEarthframe(velocity, control_state.theta, apply_ground_effect, state.z + _uav.getWheelOffset());
    std::array<double, 2> thrust_earthaxis = Aerodynamics::rotateFromBody2Earthframe({control_state.thrust, 0}, -control_state.theta);
    u[0] += thrust_earthaxis[0];
    u[1] += thrust_earthaxis[1];

    return u;
}

    void Control::applyBoundaries(std::array<double, 2> &forces, double altitude){
        
        if ( (forces[1] < _uav.getTotalMass()*9.81) && (altitude < 0.01) && (_current_control_mode == TAKEOFF)){
            forces[1] = _uav.getTotalMass()*9.81;
        }
    }

    bool Control::checkTermination(const State &state, std::array<double, 2> u){
        switch (_current_control_mode){
            case TAKEOFF:
            if (u[1] > 1.3*_uav.getTotalMass())
                return true;
            break;
        }

        switch(_termination_mode){
            case REACH_CEIL:
            if (state.z >= _ceil)
                return true;
            break;
        }
        return false;
    }



