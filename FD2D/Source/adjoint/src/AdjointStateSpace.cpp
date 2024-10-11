#include "AdjointStateSpace.h"
#include "trapz_step.h"


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

    

    return xdot_adjoint;
}


void AdjointStateSpace::solveAdjoint(AdjointControl& adj_control, StateHistory& primal_state_history, StateHistory& adjoint_state_history){
    std::cout << "solve adjoint called " << std::endl;
    int timesteps = primal_state_history.getSize();
    double ts = primal_state_history.getStartTime();
    double tf = primal_state_history.getLastTime();

    int current_timestep = timesteps;

    double tprev = tf;
    double t = tf;
    std::array<double, nargs> args = {};
    args[0] = adj_control._control._uav.getTotalMass();



    std::cout << "check if this is correct " << std::endl;
    ControlState cstate = primal_state_history.getControlStateAtTimestep(current_timestep);
    State xstate_primal = primal_state_history.getStateAtTimestep(current_timestep);

    adjoint_state_history.prepareHistoryMatrix(timesteps, tf, State(), cstate); 
    std::array<double, _neq> x = {0, 0, 0 ,0}; //adjoint has final states = 0


    while (current_timestep > 0){
        t = primal_state_history.getTimeAtTimestep(current_timestep - 1);
        double dt = t-tprev;
        std::array<double, _nin> cvars = {cstate.theta, cstate.thrust};

        std::array<double, 4>  xprimal = xstate_primal.getStatesArray();
        //caclulate AdjointControl
        std::vector<double> F_u = adj_control.F_u(xprimal, cvars);
        std::vector<double> J_u = adj_control.J_u(xprimal, cvars);
        for (int i=0; i<F_u.size(); ++i)
            args[i + 1] = F_u[i];
        for (int i=0; i<J_u.size(); ++i)
            args[F_u.size() + i + 1] = J_u[i];

        #ifdef DEBUG
            if (VERBOSITY_LEVEL >=3){

            AdjointControl::printJacobian(F_u, 2, 4);
            AdjointControl::printJacobian(J_u, 1, 4);

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
        xstate_primal = primal_state_history.getStateAtTimestep(current_timestep);
        cstate = primal_state_history.getControlStateAtTimestep(current_timestep);

        adjoint_state_history.appendState(t, computed_state, cstate);
        tprev = t;

    }

}


std::array<double, AdjointStateSpace::_nb> AdjointStateSpace::getAdjointSensitivities(
    AdjointControl& adj_control, StateHistory& primal_state_history, StateHistory& adjoint_state_history){
    int timesteps = primal_state_history.getSize();
    double ts = primal_state_history.getStartTime();
    double tf = primal_state_history.getLastTime();

    int prim_timestep = 0;
    int adj_timestep = timesteps;


    std::array<double, _nb> integral_Jb = {};
    std::array<double, _nb> integral_Rb_xadj = {};

    int t0;
    int t1 = ts;

    std::array<double, _neq> x_adj_0 = {};
    std::array<double, _neq> x_prim_0 = {};
    std::array<double, _neq> x_adj_1 = (adjoint_state_history.getStateAtTimestep(adj_timestep)).getStatesArray();
    std::array<double, _neq> x_prim_1 = (primal_state_history.getStateAtTimestep(prim_timestep)).getStatesArray();

    std::array<double, _nin> control_0= {};
    std::array<double, _nin> control_1 = (primal_state_history.getControlStateAtTimestep(prim_timestep)).getControlStatesArray();
    

    std::vector<double> Jb_0(_nb);
    std::vector<double> Jb_1 = adj_control.J_b(x_prim_1, control_1, t1);

    std::vector<double> Rbtemp(_neq*_nin);
    Rbtemp = adj_control.R_b(x_prim_1, control_1, t1);
    std::vector<double> Rb_xadj_0(_nb);
    std::vector<double> Rb_xadj_1(_nb);
    //Rb_xadj_1 = AdjointControl::multiplyJacWithVect(Rbtemp,  x_adj_1);

    Rb_xadj_1 = AdjointControl::multiplyVectWithJac(x_adj_1, Rbtemp);
    
    while (prim_timestep<timesteps){
        //assign previous timestep values
        t0 = t1;
        x_adj_0 = x_adj_1;
        x_prim_0 = x_prim_1;
        control_0 = control_1;
        Jb_0 = Jb_1;
        Rb_xadj_0 = Rb_xadj_1;

        //assign next timestep values
        t1 = primal_state_history.getTimeAtTimestep(prim_timestep + 1);

        x_adj_1 = (adjoint_state_history.getStateAtTimestep(adj_timestep - 1)).getStatesArray();
        x_prim_1 = (primal_state_history.getStateAtTimestep(prim_timestep + 1)).getStatesArray();


        control_1 = (primal_state_history.getControlStateAtTimestep(prim_timestep + 1)).getControlStatesArray();

        Jb_1 = adj_control.J_b(x_prim_1, control_1, t1);


        Rbtemp = adj_control.R_b(x_prim_1, control_1, t1); //4x4
        //Rb_xadj_1 = AdjointControl::multiplyJacWithVect(Rbtemp,  x_adj_1); //4x1
        Rb_xadj_1 = AdjointControl::multiplyVectWithJac(x_adj_1, Rbtemp); //1x4


        for (int i=0; i < _nb; ++i){
            integral_Jb[i] += trapz({static_cast<double>(t0), static_cast<double>(t1)}, {Jb_0[i], Jb_1[i]});
            integral_Rb_xadj[i] += trapz({static_cast<double>(t0), static_cast<double>(t1)}, {Rb_xadj_0[i], Rb_xadj_1[i]});
        }


        prim_timestep++;
        adj_timestep--;
    }

    std::array<double, _nb> dJdb;
    for (int i=0; i < _nb; ++i){
        dJdb[i] = integral_Jb[i] + integral_Rb_xadj[i];
    }


    return dJdb;
}

double AdjointStateSpace::evaluateLossFunctional(AdjointControl& adj_control, StateHistory& primal_state_history){
    int timesteps = primal_state_history.getSize();
    double ts = primal_state_history.getStartTime();
    double tf = primal_state_history.getLastTime();

    int timestep = 0;
    int t0;
    int t1 = ts;
    std::array<double, _neq> x_prim_0 = {};
    std::array<double, _neq> x_prim_1 = (primal_state_history.getStateAtTimestep(timestep)).getStatesArray();


    double J0 = 0;
    double J1 = adj_control.evaluateLossFunction(x_prim_1);

    double integral_J = 0;
    while (timestep<timesteps){
        //assign previous timestep values
        t0 = t1;
        x_prim_0 = x_prim_1;
        J0 = J1;

        //assign next timestep values
        t1 = primal_state_history.getTimeAtTimestep(timestep + 1);

        x_prim_1 = (primal_state_history.getStateAtTimestep(timestep + 1)).getStatesArray();


        J1 = adj_control.evaluateLossFunction(x_prim_1);


        integral_J += trapz({static_cast<double>(t0), static_cast<double>(t1)}, {J0, J1});
        


        std::cout << "timestep = " << timestep << std::endl;
        timestep++;
    }

    return integral_J;
}