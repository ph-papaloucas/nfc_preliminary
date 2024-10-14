#include "maneuver_mission.h"
#include <tuple>

std::array<double, 2> initializeVelocity(std::array<double, 2> vel_prev, double gamma){
    double vel_norm = sqrt(pow(vel_prev[1],2) + pow(vel_prev[0],2));
    std::array<double,2> vel = {vel_norm, 0};
    double err = 1e-6;
    while ( abs((atan2(vel[1], vel[0]) - gamma)) > err ){
        vel[1] = sqrt(pow(vel_norm,  2)-pow(vel[0], 2))*tan(gamma);
        vel[0] = sqrt(pow(vel_norm,2) - pow(vel[1],2));
    }

    return vel;

}

void maneuver_mission(){

    std::tuple<UAV, EngineMap, std::vector<double>>tupleReturn= createAircraftEnigneFromTaskDat();
    UAV uav = std::get<0>(tupleReturn);
    EngineMap engine= std::get<1>(tupleReturn);
    std::vector<double> parameters = std::get<2>(tupleReturn);
    double gamma_climb = parameters[0];

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
    std::array<double, 2> init_input = {0,0};
    std::array<double, 2> init_cstate = {0,0};
    control.setControlMode(Control::TAKEOFF);
    state = State(init_state, init_input);
    cstate = ControlState(init_cstate);
    sim_maneuver.setParameters(time, dt, max_timesteps);
    sim_maneuver.solve(control, state, cstate, takeoff_h);
    takeoff_h.printToCsv(".outData/takeoff");

    ////////////// Climb segment //////////////
    control.setControlMode(Control::TRIM_GAMMA);
    state = takeoff_h.getStateAtTimestep(takeoff_h.getSize());   
    cstate = takeoff_h.getControlStateAtTimestep(takeoff_h.getSize());
    // dictate climb angle by initializing the initial velocity accordingly
    std::array<double,2> vel = initializeVelocity({state.u, state.w}, gamma_climb);
    state.u = vel[0];
    state.w = vel[1];
    sim_maneuver.setParameters(takeoff_h.getLastTime(), dt, max_timesteps);
    sim_maneuver.solve(control, state, cstate, climb_h);
    climb_h.printToCsv(".outData/climb");

    // ////////////// Cruise segment //////////////
    // control.setControlMode(Control::TRIM_GAMMA);
    // state = climb_h.getStateAtTimestep(climb_h.getSize());
    // cstate = climb_h.getControlStateAtTimestep(climb_h.getSize());
    //  // dictate climb angle by initializing the initial velocity accordingly
    // vel = initializeVelocity({state.u, state.w}, gamma_climb);
    // state.u = vel[0];
    // state.w = vel[1];
    // sim_maneuver.setParameters(climb_h.getLastTime(), dt, max_timesteps);
    // sim_maneuver.solve(control, state, cstate, maneuver1_h);
    // maneuver1_h.printToCsv(".outData/maneuver1_h");
    std::cout << "end " << std::endl;
}