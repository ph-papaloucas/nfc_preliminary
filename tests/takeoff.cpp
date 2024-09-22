#include "UAV.h"
#include "EngineMap.h"
#include "StateSpace.h"
#include "State.h"
#include "Control.h"
#include <iostream>

int main(){
    std::cout <<"Test executable for takeoff (classes UAV, Aerodynamics, State, StateSpace\n";

    double total_mass = 1;
    UAV p1(0.5, 9, total_mass, 0.01, 1);

    StateHistory history;
    StateSpace sim(history);
    sim.setParameters(0, 0.1, 20);
    State init_state;
    ControlState control_state;
    EngineMap engine(EngineMap::P12x6);


    Control control(engine, p1, 30);
    control.setControlMode(Control::TAKEOFF);

    std::cout << "Solving takeof for initial state: " << init_state << std::endl;
    sim.solve(p1, init_state, control, control_state);

    std::cout << history << std::endl;

    return 0;
}