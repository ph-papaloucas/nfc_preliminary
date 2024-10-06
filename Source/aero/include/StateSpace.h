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
	StateSpace(StateHistory& state_history);
	void setParameters(const double& t0, const double& dt, const int& total_timesteps);
	void solve(UAV& uav, State& initial_state, Control& control, ControlState& control_state);
	

	bool terminate_at_takeoff = false;
	bool trim_for_takeoff = false;
	bool apply_rolling_resistance = false;

private:
	static const size_t _neq = 4;
	static const size_t _nin = 2;
	double _t0, _dt;
	int _total_timesteps;

	static std::array<double, 4> _eom(const std::array<double, 4>& x, const std::array<double, 2>& u, const std::array<double, 1>& m);

	StateHistory& _state_history;
	static const int _timesteps_to_update_screen = 10;
	static bool _updateScreen(int timestep);
	//Control& _control;
};