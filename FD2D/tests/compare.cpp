#include "UAV.h"
#include "EngineMap.h"
#include "StateSpace.h"
#include "State.h"
#include "Control.h"
#include <iostream>

int main(){
    std::cout <<"Test executable for climb (classes UAV, Aerodynamics, State, StateSpace\n";

    double S = 0.453529;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(0);
    double mass = 3.05813;
    UAV p1(S, AR, mass, a0, 1);
    //p1.setAoaTakeoff(Aerodynamics::deg2rad(0));
    Aerodynamics aero(p1);


    StateHistory history;
    StateSpace sim(history);
    sim.setParameters(30.2, 0.2, 200);

//30.2,514.317,59.639,18.839,0.230919,0.012108,0.0568184
//30.4,518.106,59.6852,19.0561,0.230744,0.0119716,0.0556656
    p1.set_e(0.666837);
    State init_state({514.317,59.639,18.839,0.230919}, {0, 0});
    ControlState control_state;
    EngineMap engine(EngineMap::P12x6);


    Control control(engine, p1, 30);
    control.setControlMode(Control::TRIM_GAMMA);
    control.setTerminationMode(Control::REACH_CEIL);


    std::cout << "Solving climb for initial state: " << init_state << std::endl;
    sim.solve(p1, init_state, control, control_state);
 
    history.printToCsv("compare/Aircraft_1_statehistory");
    return 0;
}