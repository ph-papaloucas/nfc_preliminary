    
#include "UAV.h"
#include "EngineMap.h"
#include "StateSpace.h"
#include "State.h"
#include "Control.h"
#include <iostream>

int main(){

    double S = 0.5;
    double AR = 9;
    double a0 = Aerodynamics::deg2rad(-4);
    double mass = 3;
    UAV p1(S, AR, mass, a0, 1);
    //p1.setAoaTakeoff(Aerodynamics::deg2rad(0));
    Aerodynamics aero(p1);

    EngineMap engine(EngineMap::P12x6);
    State init_state({0, 10, 15, 0}, {0, 0});
    ControlState control_state;

    Control control(engine, p1, 30);
    control.setControlMode(Control::THETA, {0,0,0,0});
    control.setTerminationMode(Control::REACH_CEIL);

    double vxbody = 0;
    double vzbody = 0;
    for (int i=0; i <10; ++i){
        std::array<double, 2> v = {vxbody, vzbody};
        double thrust = control.getThrust(v);
        std::cout << "vxbody = " << vxbody << " thrust = " << thrust << std::endl;
        vxbody = i*2;
    }


    

    double th = 0;
    // for (int i=0; i <10; ++i){
    //     control.setControlMode(Control::THETA, {th,0,0,0});
    //     State state({0,0, 10, 0}, {0, 30});
    //     std::array<double, 2> F = control.getForces(state, control_state, 0);
    //     std::cout << "theta:" << th << " --> X= " << F[0] << " Z = " << F[1] << std::endl;
    //     th = Aerodynamics::deg2rad(i*5);
    // }
2.300000,57.147189,38.512351,49.840232,42.340927,166.932857,228.833460,0.652277,

    th = 0.46;
    control.setControlMode(Control::THETA, {th,0,0,0});
    State state({30,20, 27, 15}, {1123123312, 123123132});
    std::array<double, 2> F = control.getForces(state, control_state, 0);
    control.applyBoundaries(F, state.altitude());
    std::cout << "theta:" << th << " --> X= " << F[0] << " Z = " << F[1] << std::endl;


    

}