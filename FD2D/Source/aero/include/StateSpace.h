#pragma once

#include "UAV.h"
#include "Aerodynamics.h"
#include "State.h"
#include "Control.h"
#include <array>
#include <iostream>
#include "RK4.h"



class StateSpace {
public:
	StateSpace(){};
	void setParameters(const double& t0, const double& dt, const int& total_timesteps);
	void solve(Control& control, State& initial_state, ControlState& control_state, StateHistory& state_history);
	
private:
	static const size_t _neq = 4;
	static const size_t _nin = 2;
	double _t0, _dt;
	int _total_timesteps;

	static std::array<double, 4> _eom(const std::array<double, 4>& x, const std::array<double, 2>& u, const std::array<double, 1>& m);

	static const int _timesteps_to_update_screen = 10;
	static bool _updateScreen(int timestep);
};