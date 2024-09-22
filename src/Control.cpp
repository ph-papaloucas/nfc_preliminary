#include "Control.h"

Control::Control():_max_amps(0), _current_thrust_mode(MAX){};
Control::Control(EngineMap engine, const UAV& uav, double max_amps):_engine(engine),_uav(uav), _aero(uav), _max_amps(max_amps), _current_thrust_mode(MAX){};

double Control::_getThrust(std::array<double, 2> velocity_bodyframe){
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
double Control::_getGamma(std::array<double, 2> velocity){
    double gamma = 0;
    switch (_current_control_mode){
        case TRIM:
            gamma =  _aero.getGammaForTrim(velocity);
        case TAKEOFF:
            gamma =  _uav.getAoaTakeoff();
    }
    return gamma;
}

void Control::_applyControl(ControlState &control_state, std::array<double,2 > velocity){
    control_state.gamma = _getGamma(velocity);
    control_state.thrust = _getThrust(Aerodynamics::rotateVector2Bodyframe(velocity, control_state.gamma));


}

std::array<double, 2> Control::getForces(const State &state, ControlState& control_state){
    std::array<double, 2> velocity = {state.u, state.w};
    _applyControl(control_state, velocity);
    std::array<double,2 > u = _aero.getForcesBodyframe(Aerodynamics::rotateVector2Bodyframe(velocity, control_state.gamma));
    u[0] += control_state.thrust;

    u = Aerodynamics::rotateVector2Earthframe(u, control_state.gamma);


    _applyGroundEffect(u);
    _applyBoundaries(u, state.z);
    return u;
}
    void Control::_applyGroundEffect(std::array<double, 2> &forces){
        std::cerr << "Control::applyGroundEffect isnt implemented yet...\n";
        std::cerr << "Control::applyGroundEffect isnt implemented yet...\n";
    }

    void Control::_applyBoundaries(std::array<double, 2> &forces, double altitude){
        
        if ( (forces[1] < _uav.getTotalMass()*9.81) && (altitude < 0.01) && (_current_control_mode == TAKEOFF)){
            forces[1] = 0e0;
        }
    }



