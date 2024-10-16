#pragma once
#include <array>
#include <iostream>


// neq = number of equations = number of states
// nin = number of inputs
// nargs = number of other arguments

template<std::size_t neq, std::size_t nin, std::size_t nargs>
using DynFunc = std::array<double, neq> (*)(
    const std::array<double, neq>&, 
    const std::array<double, nin>&, 
    const std::array<double, nargs>&);



/* example of how you should define a function that can be passed to RK4step:*/
// std::array<double, 4> eom(const std::array<double, 4>& x, const std::array<double, 2>& u, const std::array<double, 1>& args){


//     std::array<double, 4> xdot = {};
//     xdot[0] = x[2];
// 	xdot[1] = x[3];
// 	xdot[2] = u[0] / args[0];
// 	xdot[3] = u[1] / args[0];

//     return xdot;
// }


// template <std::size_t neq, std::size_t nin, std::size_t nargs>
// double RK4step(DynFunc<neq, nin, nargs> func) {
//     std::cout << "RK4 step executed" << std::endl;
//     return 0.0; // Replace with actual RK4 logic
// }





template <std::size_t neq, std::size_t nin, std::size_t nargs>
std::array<double, neq> stepRK4(DynFunc<neq, nin, nargs> dyn_func,
                                     const double t, const double dt,
                                     const std::array<double, neq> &x_prev,
                                     const std::array<double, nin> &u,
                                     const std::array<double, nargs> &args) {
  std::cout << "startstep\n";
  // RK4 coeffs
  static const std::array<double, 4> RK4 = {1.0 / 6.0, 2.0 / 6.0, 2.0 / 6.0,
                                            1.0 / 6.0};

  std::array<std::array<double, neq>, neq> K = {};

  // placeholders
  double t_2 = t + 0.5 * dt;
  double t_3 = t + dt;

  // at each timestep, forward iterations
  std::array<double, neq> x_temp = x_prev;
  std::array<double, neq> xdot = {};

  int step = 0; // 0 -> 1
  xdot = dyn_func(x_temp, u, args);
  for (int ieq = 0; ieq < neq; ieq++) {
    K[step][ieq] = dt * xdot[ieq];
    x_temp[ieq] = x_prev[ieq] + 0.5 * K[step][ieq];
  }

  step = 1; // 1 -> 2
  xdot = dyn_func(x_temp, u, args);
  for (int ieq = 0; ieq < neq; ieq++) {
    K[step][ieq] = dt * xdot[ieq];
    x_temp[ieq] = x_prev[ieq] + 0.5 * K[step][ieq];
  }
  step = 2; // 2 -> 3
  xdot = dyn_func(x_temp, u, args);
  for (int ieq = 0; ieq < neq; ieq++) {
    K[step][ieq] = dt * xdot[ieq];
    x_temp[ieq] = x_prev[ieq] + K[step][ieq];
  }

  step = 3; // 3->4
  xdot = dyn_func(x_temp, u, args);
  for (int ieq = 0; ieq < neq; ieq++) {
    K[step][ieq] = dt * xdot[ieq];
    x_temp[ieq] = x_prev[ieq] + RK4[0] * K[0][ieq] + RK4[1] * K[1][ieq] +
                  RK4[2] * K[2][ieq] + RK4[3] * K[3][ieq]; // update state
  }

  return x_temp;
}
