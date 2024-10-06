#include "AdjointStateSpace.h"


AdjointStateSpace::AdjointStateSpace(
    const UAV& uav,
    const StateHistory& primal_state_history, 
    StateHistory& adjoint_state_history,
    Jacobians& jac)
:_mass(uav.getTotalMass()), _primal_state_history(primal_state_history), _adjoint_state_history(adjoint_state_history), _jac(jac){


}

std::array<double, 4> AdjointStateSpace::_adjoint_eom(
const std::array<double, 4>& x_adjoint, const std::array<double, 2>& cvars, const std::array<double, nargs>& args){
    const int m=4;
    std::array<std::array<double, m>, 2> F_u;
    std::array<double, m> J_u;
    for (int i=0; i < m; ++i){
        F_u[0][i] = args[i+1];
        F_u[1][i] = args[m + i + 1];
        J_u[i] = args[2*m + i + 1];
    }

    #ifdef DEBUG
    if (VERBOSITY_LEVEL >=5){
        std::cout <<"adjoint eom jacobian \n";

        for (int i=0; i<2;++i){
            for (int j=0; j<4; ++j){
                std::cout << F_u[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout <<"adjoint eom jacobian \n";
        for (int i=0; i<4; ++i){
            std::cout << J_u[i] << " ";
        }
        std::cout << std::endl;
    }
    #endif

    std::array<double, 4> xdot_adjoint = {};
    int i =0; //row
    xdot_adjoint[i] = -(1/args[0])*(x_adjoint[2]*F_u[0][i] + x_adjoint[3]*F_u[1][i])+  J_u[i];
    i = 1;
	xdot_adjoint[i] = -(1/args[0])*(x_adjoint[2]*F_u[0][i] + x_adjoint[3]*F_u[1][i])+  J_u[i];
    i = 2;
	xdot_adjoint[i] = - x_adjoint[0] - (1/args[0])*(x_adjoint[2]*F_u[0][i] + x_adjoint[3]*F_u[1][i]) + J_u[i];
    i = 3;
	xdot_adjoint[i] = - x_adjoint[1] - (1/args[0])*(x_adjoint[2]*F_u[0][i] + x_adjoint[3]*F_u[1][i]) + J_u[i];

    for (int i=0; i < 4; ++i){
        //std::cout << xdot_adjoint[i] << std::endl;
    }
    

    return xdot_adjoint;
}


void AdjointStateSpace::solveAdjoint(){
    std::cout << "solve adjoint called " << std::endl;
    int timesteps = _primal_state_history.getSize();
    double ts = _primal_state_history.getStartTime();
    double tf = _primal_state_history.getLastTime();

    int current_timestep = timesteps;

    double tprev = tf;
    double t = tf;
    std::array<double, nargs> args = {};
    args[0] = _mass;



    std::cout << "check if this is correct " << std::endl;
    ControlState cstate = _primal_state_history.getControlStateAtTimestep(current_timestep);
    State xstate_primal = _primal_state_history.getStateAtTimestep(current_timestep);

    _adjoint_state_history.prepareHistoryMatrix(timesteps, tf, State(), cstate); 
    std::array<double, _neq> x = {0, 0, 0 ,0}; //adjoint has final states = 0


    while (current_timestep > 0){
        std::cout <<"timestep = " << current_timestep << std::endl;
        t = _primal_state_history.getTimeAtTimestep(current_timestep - 1);
        double dt = t-tprev;
        std::array<double, _nin> cvars = {cstate.theta, cstate.thrust};

        std::array<double, 4>  xprimal = xstate_primal.getStatesArray();
        //caclulate Jacobians
        std::vector<double> F_u = _jac.F_u(xprimal, cvars);
        std::vector<double> J_u = _jac.J_u(xprimal, cvars);
        for (int i=0; i<F_u.size(); ++i)
            args[i + 1] = F_u[i];
        for (int i=0; i<J_u.size(); ++i)
            args[F_u.size() + i + 1] = J_u[i];

        #ifdef DEBUG
            if (VERBOSITY_LEVEL >=3){

            Jacobians::printJacobian(F_u, 2, 4);
            Jacobians::printJacobian(J_u, 1, 4);

            std::cout << "args = ";
            for (int i=0; i < nargs; ++i){
                std::cout << args[i] << std::endl;
            }
        }
        #endif

        x =  stepRK4(_adjoint_eom, t, dt, x, cvars, args);
        State computed_state(x, {0, 0});

        //update things
        current_timestep --;
        xstate_primal = _primal_state_history.getStateAtTimestep(current_timestep);
        cstate = _primal_state_history.getControlStateAtTimestep(current_timestep);

        _adjoint_state_history.appendState(t, computed_state, cstate);
        tprev = t;

    }
    std::cout << "test " << std::endl;

}