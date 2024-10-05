#include "StateSpace.h"


StateSpace::StateSpace(StateHistory& state_history): _state_history(state_history) {};

void StateSpace::setParameters(const double& t0,const double& dt,const int& total_timesteps) {
	_t0 = t0;
	_dt = dt;
	_total_timesteps = total_timesteps;
}


std::array<double, 4> StateSpace::_eom(const std::array<double, 4>& x, const std::array<double, 2>& u, const std::array<double, 1>& m){


    std::array<double, 4> xdot = {};
    xdot[0] = x[2];
	xdot[1] = x[3];
	xdot[2] = u[0] / m[0];
	xdot[3] = u[1] / m[0];

    return xdot;
}



bool StateSpace::_updateScreen(int timestep){
    if ( (timestep%_timesteps_to_update_screen) == 0)
        return true;
    return false;
}


void StateSpace::solve(UAV& uav, State& initial_state, Control& control, ControlState& control_state){

    #ifdef DEBUG
        if (VERBOSITY_LEVEL>=1)
            std::cout << "SOLVING FOR INITIAL STATE: \n" << initial_state << std::endl;
        #endif
    //initial values
    double t = _t0;
    int timestep = 0;
    std::array<double, _neq> x = initial_state.getStatesArray();
    std::array<double, _nin> u = initial_state.getForcesArray();
    State computed_state = initial_state;
    _state_history.prepareHistoryMatrix(_total_timesteps, _t0, computed_state, control_state);

    std::array<double, 1> args = {uav.getTotalMass()};

    while (timestep < _total_timesteps) {
        timestep++;
        //get input
        
        u = control.getForces(computed_state, control_state);
        if (control.checkTermination(computed_state, u)){
            #ifdef DEBUG
                if (VERBOSITY_LEVEL >=1){
                    std::cout << "TERMINATING RK4 \n";
                    std::cout << "Fz = " << u[1] << std::endl;
                }
            #endif

            break; //to terminate RK4
        }
        control.applyBoundaries(u, computed_state.z);


        //solve RK4
        std::array<double, _neq> x =  stepRK4(_eom, t, _dt, x, u, args);
        computed_state = State(x, u);


        //update everything
        t += _dt;
        _state_history.appendState(t, computed_state, control_state);
        x = computed_state.getStatesArray(); //x is the previous state that will be passed to RK4

       //check if it crashed
        if (computed_state.altitude() <= -0.1)
        {
            std::cout << "altitude is zero. UAV Crashed\n";
            break;
        }



    }
}
