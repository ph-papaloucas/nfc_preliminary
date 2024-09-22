#include "StateSpace.h"


StateSpace::StateSpace(StateHistory& state_history): _state_history(state_history) {};

void StateSpace::setParameters(const double& t0,const double& dt,const int& total_timesteps) {
	_t0 = t0;
	_dt = dt;
	_total_timesteps = total_timesteps;
}


std::array<double, 4> StateSpace::_eom(const std::array<double, 4>& x, const std::array<double, 2>& u, const double mass){


    std::array<double, 4> xdot = {};
    xdot[0] = x[2];
	xdot[1] = x[3];
	xdot[2] = u[0] / mass;
	xdot[3] = u[1] / mass;

    return xdot;
}



bool StateSpace::_updateScreen(int timestep){
    if ( (timestep%_timesteps_to_update_screen) == 0)
        return true;
    return false;
}


void StateSpace::solve(UAV& uav, State& initial_state, Control& control, ControlState& control_state){
    //initial values
    double t = _t0;
    int timestep = 0;
    std::array<double, 4> x = initial_state.getStatesArray();
    std::array<double, 2> u = initial_state.getForcesArray();
    State computed_state = initial_state;
    _state_history.prepareHistoryMatrix(_total_timesteps, _t0, computed_state, control_state);

    while (timestep < _total_timesteps) {
        timestep++;

        //get input
        
        u = control.getForces(computed_state, control_state);
        std::cout << "u = " << u[0] << " " << u[1] << std::endl;


        // // terminate at takeoff
        // if (terminate_at_takeoff){
        //     double Lift = u[1];
        //     double Weight = uav.getTotalMass()*9.81;

        //     if (StateSpace::_updateScreen(timestep))
        //         std::cout << "Lift/Weight = " << Lift/Weight << std::endl;

        //     if (Lift >= 1.3*Weight){
        //         std::cout << "\nTerminate Case 1, terminate when Lift = 1.3Weight" << std::endl;
        //         std::cout << "Lift is " << Lift/Weight << "times higher than Weight " << std::endl;
        //         break;
        //     }
        //     if (computed_state.z >= 0.1 ){
        //         std::cout << "\nTerminate Case 2, terminate when leave the floor" << std::endl;
        //         std::cout << "If you didnt trim for takeoff, you might never get the Termination case 1: Lift = 1.3Weight\n";
        //         std::cout << "Lift is " << Lift/Weight << "times higher than Weight " << std::endl;
        //         break;
        //     }
        
        // }



        //solve RK4
        computed_state  = _stepRK4(t, uav, x, u, control);

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


    State StateSpace::_stepRK4(const double t,UAV& uav, const std::array<double, 4>& x_prev, const std::array<double, 2>& u, Control& control){
    //RK4 constants
    double a0 = 1.0 / 6.0;
    double a1 = 2.0 / 6.0;
    double a2 = 2.0 / 6.0;
    double a3 = 1.0 / 6.0;

    std::vector<std::vector<double>> K(4, std::vector<double>(_neq));

    //placeholders
    double t_2 = t + 0.5 * _dt;
    double t_3 = t + _dt;

    //at each timestep, forward iterations
    std::array<double, 4> x_temp = x_prev;



    int step = 0;
    double mass = uav.getTotalMass();
    for (int ieq = 0; ieq < _neq; ieq++) {
        K[step][ieq] = _dt * _eom(x_temp, u, mass)[ieq];
    }
    for (int ieq = 0; ieq < _neq; ieq++) {
        x_temp[ieq] = x_prev[ieq] + 0.5 * K[step][ieq];
    }

    step = 1;
    for (int ieq = 0; ieq < _neq; ieq++) {
        K[step][ieq] = _dt * _eom(x_temp, u, mass)[ieq];
    }
    for (int ieq = 0; ieq < _neq; ieq++) {
        x_temp[ieq] = x_prev[ieq] + 0.5 * K[step][ieq];
    }

    step = 2;
    for (int ieq = 0; ieq < _neq; ieq++) {
        K[step][ieq] = _dt * _eom(x_temp, u, mass)[ieq];
    }
    for (int ieq = 0; ieq < _neq; ieq++) {
        x_temp[ieq] = x_prev[ieq] + K[step][ieq];
    }

    step = 3;
    for (int ieq = 0; ieq < _neq; ieq++) {
        K[step][ieq] = _dt * _eom(x_temp, u,mass)[ieq];
    }


    //update state
    for (int ieq = 0; ieq < _neq; ieq++) {
        x_temp[ieq] = x_prev[ieq] + a0 * K[0][ieq] + a1 * K[1][ieq] + a2 * K[2][ieq] + a3 * K[3][ieq];
    }

    return State(x_temp, u);
}