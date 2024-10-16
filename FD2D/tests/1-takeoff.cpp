#include "Control.h"
#include "EngineMap.h"
#include "State.h"
#include "StateSpace.h"
#include "UAV.h"
#include <iostream>


int main() {
  std::cout << "Test executable for takeoff (classes UAV, Aerodynamics, State, "
               "StateSpace\n";

  double S = 0.5;
  double AR = 9;
  double a0 = Aerodynamics::deg2rad(-4);
  double mass = 10;
  UAV p1(S, AR, mass, a0, 1);
  // p1.setAoaTakeoff(Aerodynamics::deg2rad(0));
  Aerodynamics aero(p1);

  StateHistory history;
  StateSpace sim(history);
  sim.setParameters(0, 0.1, 100);
  State init_state;
  ControlState control_state;
  EngineMap engine(EngineMap::P12x6);

  Control control(engine, p1, 30);
  control.setControlMode(Control::TAKEOFF);

  std::cout << "Solving takeof for initial state: " << init_state << std::endl;
  sim.solve(p1, init_state, control, control_state);
  // sim.solve2(p1, init_state, control, control_state);

  history.printToCsv("takeoff_test");
  // history2.printToCsv("RK4new");

  return 0;
}