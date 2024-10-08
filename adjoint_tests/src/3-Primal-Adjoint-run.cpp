#include "UAV.h"
#include "EngineMap.h"
#include "StateSpace.h"
#include "State.h"
#include "Control.h"
#include <iostream>

#include "AdjointStateSpace.h"
#include "Jacobians.h"

int main(){
    std::cout <<"Test executable for adjoint iterator (classes UAV, Aerodynamics, State, StateSpace\n";


    std::array<double, 4> b = {0.05, 0, 0, 0};
    // // Prepare objects

    double S = 0.5;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(-4);
    double mass = 3;
    UAV p1(S, AR, mass, a0, 1);
    //p1.setAoaTakeoff(Aerodynamics::deg2rad(0));
    Aerodynamics aero(p1);


    // // Prepare Primal
    StateHistory history_primal;
    State init_state({0, 0, 8, 3}, {0, 0});
    ControlState control_state;
    EngineMap engine(EngineMap::P12x6);

    Control control(engine, p1, 30);
    control.setControlMode(Control::THETA, b);
    control.setTerminationMode(Control::REACH_CEIL);


    // // Solve Primal
    StateSpace sim(history_primal);
    sim.setParameters(0, 0.1, 100);
    std::cout << "Solving Primal for initial state: " << init_state << std::endl;
    sim.solve(p1, init_state, control, control_state);
 
    history_primal.printToCsv("climb_primal");


    /* If you want to check only some timesteps, uncoment this */
    /////////////////////////////////////////////////////////////
    // StateHistory history_primal2;
    // int timesteps = 4 ;
    // history_primal2.prepareHistoryMatrix(
    //     timesteps, history_primal.getStartTime(), history_primal.getStateAtTimestep(0), history_primal.getControlStateAtTimestep(0));

    // for (int i=0; i <4; ++i){
    //     history_primal2.appendState(history_primal.getTimeAtTimestep(i + 1),
    //     history_primal.getStateAtTimestep(i + 1), 
    //     history_primal.getControlStateAtTimestep(i + 1));
    // }
    // history_primal = history_primal2;
    ////////////////////////////////////////////////////////////

    // // Solve Adjoint
    StateHistory history_adjoint;
    Jacobians jac(p1, engine.getEngineCoeffs(), b, history_primal.getStartTime(), history_primal.getLastTime());
    //AdjointStateSpace adjsim(p1, history_primal, history_adjoint, jac);
    AdjointStateSpace adjsim(p1, history_primal, history_adjoint, jac);
    adjsim.solveAdjoint();
    history_adjoint.printToCsv("climb_adjoint");


    return 0;
}