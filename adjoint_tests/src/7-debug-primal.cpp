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


    std::array<double, 4> b = {0.05, 0.261860, 0, 0};
    // // Prepare objects

    double S = 0.5;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(0);
    double mass = 3;
    UAV p1(S, AR, mass, a0, 1);
    //p1.setAoaTakeoff(Aerodynamics::deg2rad(0));
    Aerodynamics aero(p1);


    int round = 0;
    double s=0.001;
    EngineMap engine(EngineMap::P12x6);
    State init_state({0, 10, 15, 0}, {0, 0});

    round++;
    // // Prepare Primal
    StateHistory history_primal;
    ControlState control_state;

    Control control(engine, p1, 30);
    control.setControlMode(Control::THETA, b);
    control.setTerminationMode(Control::REACH_CEIL);


    // // Solve Primal
    StateSpace sim(history_primal);
    sim.setParameters(0, 0.1, 100);
    std::cout << "Solving Primal for initial state: " << init_state << std::endl;
    sim.solve(p1, init_state, control, control_state);

    history_primal.printToCsv("prim/debugme");


    // // Solve Adjoint
    StateHistory history_adjoint;
    Jacobians jac(p1, engine.getEngineCoeffs(), b, history_primal.getStartTime(), history_primal.getLastTime());
    AdjointStateSpace adjsim(p1, history_primal, history_adjoint, jac);
    // AdjointStateSpace adjsim(p1, history_primal, history_adjoint, jac);
    // adjsim.solveAdjoint();

    
    // history_adjoint.printToCsv("adj/climb_opt_"+  std::to_string(round));


    //std::array<double, 4> sens = adjsim.getAdjointSensitivities();
    double loss = adjsim.evaluateLossFunctional();
    //evaluate Jacobean at b
    std::cout << "Loss Function = " << loss;



}