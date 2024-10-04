#pragma once

#include "UAV.h"
#include "Aerodynamics.h"
#include "State.h"
#include "Control.h"
#include <array>
#include <iostream>




class StateSpace {
public:
	StateSpace(StateHistory& state_history);
	void setParameters(const double& t0, const double& dt, const int& total_timesteps);
	void solve(UAV& uav, State& initial_state, Control& control, ControlState& control_state);
	

	bool terminate_at_takeoff = false;
	bool trim_for_takeoff = false;
	bool apply_rolling_resistance = false;

private:
	const int _neq = 3;
	const int _nu = 6;
	double _t0, _dt;
	int _total_timesteps;

	State _stepRK4(const double t,UAV& uav, const std::array<double, 4>& x_prev, const std::array<double, 2>& u, Control& control);
	std::array<double, 4> _eom(const std::array<double, 4>& x, const std::array<double, 2>& u, const double mass);


	StateHistory& _state_history;
	static const int _timesteps_to_update_screen = 10;
	static bool _updateScreen(int timestep);
	//Control& _control;
};