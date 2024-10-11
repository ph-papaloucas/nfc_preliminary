#include "aero_headers.h"
#include "Scoring.h"
#include "AdjointControl.h"
#include "AdjointStateSpace.h"
UAV createAirplane(){
    double S = 0.453529;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(0);
    double mass = 3.05813;
    UAV p1(S, AR, mass, a0, 1);

    return p1;
}

EngineMap createEngine(){
    EngineMap engine(EngineMap::P12x6);

    return engine;
}


void climbNormal(Control& control, StateHistory& takeoff_h, StateHistory& climb_h, StateSpace& sim_maneuver, double dt, double timesteps){
    control.setControlMode(Control::TRIM_GAMMA, {Aerodynamics::deg2rad(30), 0, 0, 0});
    State state = takeoff_h.getStateAtTimestep(takeoff_h.getSize());
    double velnorm = sqrt(pow(state.u,2 ) + pow(state.w,2));
    state.w = velnorm/5;
    state.u = sqrt( pow(velnorm,2) - pow(state.w,2) );
    
    ControlState cstate = takeoff_h.getControlStateAtTimestep(takeoff_h.getSize());
    std::cout << state << std::endl;
    sim_maneuver.setParameters(takeoff_h.getLastTime(), dt, timesteps);
    sim_maneuver.solve(control, state, cstate, climb_h);
}


void climbAdjoint(Control& control, StateHistory& takeoff_h, StateHistory& climb_h, StateSpace& sim_maneuver, double dt, double timesteps){

    int round = 0;
    State state = takeoff_h.getStateAtTimestep(takeoff_h.getSize());
    ControlState cstate = takeoff_h.getControlStateAtTimestep(takeoff_h.getSize());
    sim_maneuver.setParameters(takeoff_h.getLastTime(), dt, timesteps);
    std::array<double, 4> b = {0.1, 0,  0, 0};
    std::array<double, 4> s = {0.1, 0, 0, 0};

    double prev_loss = 123123;
    double dloss = 0.000001;
    while (round<20) {
        control.setControlMode(Control::THETA, b);

        //solve primal
        StateHistory primal_h;
        std::cout << state << std::endl;
        sim_maneuver.solve(control, state, cstate, primal_h);
        primal_h.printToCsv(".outData/primal/climb-" + std::to_string(round));


        //solve adjoint
        StateHistory adj_h;
        AdjointControl adj_control(control, b, primal_h.getStartTime(), primal_h.getLastTime());
        AdjointStateSpace adjsim;
        adjsim.solveAdjoint(adj_control, primal_h, adj_h);

        
        adj_h.printToCsv(".outData/adjoint/climb-"+  std::to_string(round));


        std::array<double, 4> sens = adjsim.getAdjointSensitivities(adj_control, primal_h, adj_h);
        double loss = adjsim.evaluateLossFunctional(adj_control, primal_h);
        double err = abs( abs(loss)-abs(prev_loss)) ;
        if ( err  < dloss  ){
            break;
        }
        prev_loss = loss;
        //evaluate Jacobean at b
        std::cout << "Loss Function = " << loss;
        for (int i=0; i < b.size() ; ++i){
            b[i] = b[i] - s[i]*sens[i];
        }
        round++;
    }

     //solve primal
    sim_maneuver.solve(control, state, cstate, climb_h);
}
 

int main(){

    UAV p1 = createAirplane();
    EngineMap engine = createEngine();
    Control control(p1,engine, 30);
    std::array<double, 4> init_state = {0, 0, 0, 0};
    std::array<double, 2> init_input = {0,0};
    std::array<double, 2> init_cstate = {0,0};
    double time = 0;
    int timesteps = 50;
    // General 
    double dt = 0.1;

    /// TAKEOFF SEGMENT
    control.setControlMode(Control::TAKEOFF);
    StateHistory takeoff_h;
    StateSpace sim_maneuver;
    State state(init_state, init_input);
    ControlState cstate(init_cstate);
    sim_maneuver.setParameters(time, dt, timesteps);

    sim_maneuver.solve(control, state, cstate, takeoff_h);
    takeoff_h.printToCsv(".outData/takeoff");

    // Climb segment
    StateHistory climb_h1;
    climbNormal(control, takeoff_h, climb_h1, sim_maneuver, dt, timesteps);
    StateHistory climb_h2;
    climbAdjoint(control, takeoff_h, climb_h2, sim_maneuver, dt, timesteps);

    StateHistory climb_h = climb_h1;
    climb_h.printToCsv(".outData/climb");

    //  Cruise segment
    control.setControlMode(Control::TRIM_GAMMA, {Aerodynamics::deg2rad(0), 0, 0, 0});
    StateHistory cruise_h;
    state = climb_h.getStateAtTimestep(climb_h.getSize());
    double velnorm = sqrt(pow(state.u,2 ) + pow(state.w,2));
    state.u = velnorm;
    state.w = 0;

    cstate = climb_h.getControlStateAtTimestep(climb_h.getSize());
    std::cout << state << std::endl;
    sim_maneuver.setParameters(climb_h.getLastTime(), dt, timesteps);
    sim_maneuver.solve(control, state, cstate, cruise_h);
    cruise_h.printToCsv(".outData/cruise");
    

    return 0;
}