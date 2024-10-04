#include "aero_headers.h"
#include "Scoring.h"
int main(){

    /* Variables that are not dealt with yet*/
    double maxamps = 30;

    /* Read airplane from txt file */
    double S = 0.5;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(-4);
    double mass = 10;
    double aoa_takeoff = 5;
    double rho_payload = 1;

    UAV p1(S, AR, mass, a0, rho_payload);
    p1.setAoaTakeoff(Aerodynamics::deg2rad(aoa_takeoff));


    //Prepare Objects for the simulation
    Aerodynamics aero(p1);
    StateHistory history;
    StateSpace sim(history);
    ControlState control_state;
    EngineMap engine(EngineMap::P12x6);
    Control control(engine, p1, maxamps);

    //Simulation parameters
    double t0 = 0; double dt = 0.05; int timesteps = 100;

    //Manuver1: Takeoff
    control.setControlMode(Control::TAKEOFF);
    State init_state;
    sim.setParameters(t0, dt, timesteps);

    sim.solve(p1, init_state, control, control_state);

    history.printToCsv("sim1");


//    //Manuver 2: Climb
//    control.setControlMode(Control::CONST_GAMMA);   
//    control_state = ControlState();
//    init_state.setState(0, 0, 10, 10);
//    sim.setParameters(history.getLastTime(), dt, TF);
//    sim.solve(p1, init_state, control, control_state);
   
   
   return 0;
}