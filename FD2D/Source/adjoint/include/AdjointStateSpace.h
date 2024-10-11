#pragma once


#include "State.h"
#include "AdjointControl.h"
#include <array>
#include <iostream>
#include "RK4.h"


class AdjointStateSpace {
private:
	static const size_t _neq = 4;
	static const size_t _nin = 2;
	static const size_t _nb = 4;
public:
	static const size_t nargs = 13;

	AdjointStateSpace(){};

	void solveAdjoint(AdjointControl& adj_control, StateHistory& primal_history, StateHistory& adj_history);


	std::array<double, _nb> getAdjointSensitivities(AdjointControl& adj_control, StateHistory& primal_state_history, StateHistory& adjoint_state_history);
	
	double evaluateLossFunctional(AdjointControl& adj_control, StateHistory& primal_state_history);
private:

	static std::array<double, 4> _adjoint_eom(
const std::array<double, 4>& x_adjoint, const std::array<double, 2>& cvars, const std::array<double, nargs>& args);

	// static const int _timesteps_to_update_screen = 10;
	// static bool _updateScreen(int timestep);
};