#include "UAV.h"
#include "EngineMap.h"
#include "StateSpace.h"
#include "State.h"
#include "Control.h"
#include <iostream>

int main(){
    std::cout <<"Test executable for climb (classes UAV, Aerodynamics, State, StateSpace\n";

    double S = 0.5;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(-4);
    double mass = 3;
    UAV p1(S, AR, mass, a0, 1);
    //p1.setAoaTakeoff(Aerodynamics::deg2rad(0));
    Aerodynamics aero(p1);


    StateHistory history;
    StateSpace sim(history);
    sim.setParameters(0, 0.1, 100);
    State init_state({0, 0, 20, 0}, {0, 0});
    ControlState control_state;
    EngineMap engine(EngineMap::P12x6);


    Control control(engine, p1, 30);
    control.setControlMode(Control::THETA, {Aerodynamics::deg2rad(0), 0, 0, 0});
    control.setTerminationMode(Control::REACH_CEIL);


    std::cout << "Solving climb for initial state: " << init_state << std::endl;
    sim.solve(p1, init_state, control, control_state);
 
    history.printToCsv("climb_theta_t");
    return 0;
}