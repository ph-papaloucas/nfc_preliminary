#include "maneuver_mission.h"
#include <tuple>

std::array<double, 2> initializeVelocity(std::array<double, 2> vel_prev,
                                         double vel_ratio) {
  double vel_norm = sqrt(pow(vel_prev[0], 2) + pow(vel_prev[1], 2));
  std::array<double, 2> vel = {vel_norm, 0};
  double err = 1e-6;
  double error = 10;
  while (error > err) {
    vel[1] = vel[0] * tan(vel_ratio);
    vel[0] = sqrt(pow(vel_norm, 2) - pow(vel[1], 2));
    error = abs(abs(atan2(vel[1], vel[0])) - abs(vel_ratio));
  }

  return vel;
}

void maneuver_mission() {

  std::tuple<UAV, EngineMap, std::vector<double>> tupleReturn =
      createAircraftEnigneFromTaskDat();
  UAV uav = std::get<0>(tupleReturn);
  EngineMap engine = std::get<1>(tupleReturn);
  std::vector<double> parameters = std::get<2>(tupleReturn);
  double vel_ratio_climb = parameters[0];

  Control control(uav, engine, 30);

  /// Initialization and general settings
  StateSpace sim_maneuver;
  State state;
  ControlState cstate;
  StateHistory takeoff_h;
  StateHistory climb_h;
  StateHistory maneuver1_h;
  double dt = 0.1;
  int max_timesteps = 500;

  ////////////// TAKEOFF SEGMENT //////////////
  double time = 0;
  std::array<double, 4> init_state = {0, 0, 0, 0};
  std::array<double, 2> init_input = {0, 0};
  std::array<double, 2> init_cstate = {0, 0};
  control.setControlMode(Control::TAKEOFF);
  state = State(init_state, init_input);
  cstate = ControlState(init_cstate);
  sim_maneuver.setParameters(time, dt, max_timesteps);
  sim_maneuver.solve(control, state, cstate, takeoff_h);
  takeoff_h.printToCsv(".outData/takeoff");

  ////////////// Climb segment //////////////
  control.setControlMode(Control::TRIM_GAMMA);
  control.setTerminationMode(Control::REACH_CEIL, 100);
  state = takeoff_h.getStateAtTimestep(takeoff_h.getSize());
  cstate = takeoff_h.getControlStateAtTimestep(takeoff_h.getSize());
  // dictate climb angle by initializing the initial velocity accordingly
  std::array<double, 2> vel =
      initializeVelocity({state.u, state.w}, vel_ratio_climb);
  state.u = vel[0];
  state.w = vel[1];
  sim_maneuver.setParameters(takeoff_h.getLastTime(), dt, max_timesteps);
  sim_maneuver.solve(control, state, cstate, climb_h);
  climb_h.printToCsv(".outData/climb");

  ////////////// Cruise segment //////////////
  state = climb_h.getStateAtTimestep(climb_h.getSize());
  cstate = climb_h.getControlStateAtTimestep(climb_h.getSize());
  // dictate climb angle by initializing the initial velocity accordingly
  vel = initializeVelocity({state.u, state.w}, 0.);
  state.u = vel[0];
  state.w = vel[1];
  control.setControlMode(Control::TRIM_GAMMA);
  control.setTerminationMode(Control::REACH_DISTANCE, state.x + 100);
  sim_maneuver.setParameters(climb_h.getLastTime(), dt, max_timesteps);
  sim_maneuver.solve(control, state, cstate, maneuver1_h);
  maneuver1_h.printToCsv(".outData/cruise");
  std::cout << "end " << std::endl;
}