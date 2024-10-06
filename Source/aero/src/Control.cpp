#include "Control.h"

Control::Control():_max_amps(0), _current_thrust_mode(MAX){};
Control::Control(EngineMap engine, const UAV& uav, double max_amps):_engine(engine),_uav(uav), _aero(uav), _max_amps(max_amps), _current_thrust_mode(MAX){};

double Control::getThrust(std::array<double, 2> velocity_bodyframe) const{
    double vel_wind = velocity_bodyframe[0];
    double thrust = 0 ;

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
double Control::_getTheta(std::array<double, 2> velocity){
    double theta = 0;
    #ifdef DEBUG
        if(VERBOSITY_LEVEL>=3){
            std::cout << "CONTROL MODE = " << _current_control_mode << std::endl;
        }
    #endif
    switch (_current_control_mode){
        // case TRIM:
        //     theta =  _aero.getThetaForTrim(velocity, );
        //     break;
        case TAKEOFF:
            theta =  _uav.getAoaTakeoff();
            break;
        case TRIM_GAMMA:
            theta = _aero.getThetaForTrim(velocity, *(this));
            break;

    }
    return theta;
}

void Control::_applyControl(ControlState &control_state, std::array<double,2 > velocity){
    control_state.theta = _getTheta(velocity);
    control_state.thrust = getThrust(Aerodynamics::rotateFromEarth2Bodyframe(velocity, control_state.theta));


}

std::array<double, 2> Control::getForces(const State &state, ControlState& control_state){
    std::array<double, 2> velocity = {state.u, state.w};
    _applyControl(control_state, velocity);

    bool apply_ground_effect = false; //CHANGE ME
    std::array<double, 2> u = _aero.getAeroForcesEarthframe(velocity, control_state.theta, apply_ground_effect, state.z + _uav.getWheelOffset());
    std::array<double, 2> thrust_earthaxis = Aerodynamics::rotateFromBody2Earthframe({control_state.thrust, 0}, control_state.theta);
    u[0] += thrust_earthaxis[0];
    u[1] += thrust_earthaxis[1];

    //_applyGroundEffect(u);
    return u;
}

    void Control::applyBoundaries(std::array<double, 2> &forces, double altitude){
        
        if ( (forces[1] < _uav.getTotalMass()*9.81) && (altitude < 0.01) && (_current_control_mode == TAKEOFF)){
            forces[1] = 0e0;
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



