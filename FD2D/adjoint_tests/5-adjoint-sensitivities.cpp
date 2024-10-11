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
 
    history_primal.printToCsv("climb2_primal");


    // // Solve Adjoint
    StateHistory history_adjoint;
    Jacobians jac(p1, engine.getEngineCoeffs(), b, history_primal.getStartTime(), history_primal.getLastTime());
    //AdjointStateSpace adjsim(p1, history_primal, history_adjoint, jac);
    AdjointStateSpace adjsim(p1, history_primal, history_adjoint, jac);
    adjsim.solveAdjoint();
    history_adjoint.printToCsv("climb2_adjoint");




    std::cout << " CHECK MATRIX MULTIPLICATION INSIDE JACOBIAN\n";
    std::vector<double> jactest =  {1, 2, 3, 4,
        5,6,7,8};
    std::array<double, 4> vect = {1, 2, 3, 4};

    std::vector<double> value = Jacobians::multiplyJacWithVect(jactest, vect);
    std::cout << "jac x vect \n";
    std::cout << value[0] << "should be = 30\n";
    std::cout << value[1] << "should be = 70\n";


    std::vector<double> value2 = Jacobians::multiplyVectWithJac(vect, jactest);
    std::cout << "vect x jac \n";
    std::cout << value2[0] << "should be = 50\n";
    std::cout << value2[1] << "should be = 60\n";

    std::array<double, 4> sens = adjsim.getAdjointSensitivities();









    return 0;
}