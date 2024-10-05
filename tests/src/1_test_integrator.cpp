#include <RK4.h>


#include <iostream>


std::array<double, 4> eom(const std::array<double, 4>& x, const std::array<double, 2>& u, const std::array<double, 1>& mass){


    std::array<double, 4> xdot = {};
    xdot[0] = x[2];
	xdot[1] = x[3];
	xdot[2] = u[0] / mass[0];
	xdot[3] = u[1] / mass[0];

    return xdot;
}

int main(){
    double t = 0;
    double dt = 0.5;
    std::array<double, 4> xprev = {0,0, 4,1};
    std::array<double, 2> u = {10, 0};
    std::array<double, 1> mass = {5};

    std::array<double, 4> xnext = stepRK4(eom, t, dt, xprev, u, mass);
    for (int i=0; i < 4; ++i){
        std::cout << xnext[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}