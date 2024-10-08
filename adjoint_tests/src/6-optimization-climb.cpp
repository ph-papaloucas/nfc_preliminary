#include "Control.h"
#include "EngineMap.h"
#include "State.h"
#include "StateSpace.h"
#include "UAV.h"
#include <iostream>


#include "AdjointStateSpace.h"
#include "Jacobians.h"

int main() {
  std::cout << "Test executable for adjoint iterator (classes UAV, "
               "Aerodynamics, State, StateSpace\n";

  std::array<double, 4> b = {0.05, 0.05, 0, 0};
  // // Prepare objects

  double S = 0.5;
  double AR = 9;
  double a0 = utils::deg2rad(-4);
  double mass = 3;
  UAV p1(S, AR, mass, a0, 1);
  // p1.setAoaTakeoff(Aerodynamics::deg2rad(0));
  Aerodynamics aero(p1);

  int round = 0;
  std::array<double, 4> s = {0.0001, 0.0001, 0.0001, 0.0001};
  EngineMap engine(EngineMap::P12x6);
  State init_state({0, 10, 15, 0}, {0, 0});

  std::vector<double> lossvect;
  double ts = 0;
  double dt = 0.1;
  double final_timestep = 1000;
  while (round < 15) {
    round++;
    // // Prepare Primal
    StateHistory history_primal;
    ControlState control_state;

    Control control(engine, p1, 30);
    control.setControlMode(Control::THETA, b);
    control.setTerminationMode(Control::REACH_CEIL);

    // // Solve Primal
    StateSpace sim(history_primal);
    sim.setParameters(ts, dt, final_timestep);
    std::cout << "\n--------------------Solving Primal: " << init_state
              << std::endl;
    sim.solve(p1, init_state, control, control_state);

    history_primal.printToCsv(".outData/prim/climb_opt_" +
                              std::to_string(round));

    // // Solve Adjoint
    StateHistory history_adjoint;
    Jacobians jac(p1, engine.getEngineCoeffs(), b,
                  history_primal.getStartTime(), history_primal.getLastTime());
    // AdjointStateSpace adjsim(p1, history_primal, history_adjoint, jac);
    AdjointStateSpace adjsim(p1, history_primal, history_adjoint, jac);
    std::cout << "\n--------------------Solving Adjoint: " << init_state
              << std::endl;
    adjsim.solveAdjoint();

    history_adjoint.printToCsv(".outData/adj/climb_opt_" +
                               std::to_string(round));

    std::array<double, 4> sens = adjsim.getAdjointSensitivities();
    double loss = adjsim.evaluateLossFunctional();
    // evaluate Jacobean at b
    std::cout << "Loss Function = " << loss;

    // update b
    // b[0] = b[0] - s[0]*sens[0];
    // b[1] = b[1] - s[1]*sens[1];
    // b[2] = b[2] - s[2]*sens[2];
    // b[3] = b[3] - s[3]*sens[3];
    for (int i = 0; i < b.size(); ++i) {
      b[i] = b[i] - s[i] * sens[i];
    }
    lossvect.push_back(loss);
  }

  std::cout << "LOSS HISTORY \n";
  for (int i = 0; i < lossvect.size(); ++i) {
    std::cout << i << " loss = " << lossvect[i] << std::endl;
  }

  std::cout << "Params \n";
  for (int i = 0; i < b.size(); ++i) {
    std::cout << b[i] << " ";
  }

  return 0;
}