#pragma once


#include "State.h"
#include "Jacobians.h"
#include <array>
#include <iostream>
#include "RK4.h"


class AdjointStateSpace {
public:
	static const size_t nargs = 13;

	AdjointStateSpace(
    const UAV& uav,
    const StateHistory& primal_state_history, 
	StateHistory& adjoint_state_history, Jacobians& jac);

	void solveAdjoint();
	

private:
	static const size_t _neq = 4;
	static const size_t _nin = 2;

	static std::array<double, 4> _adjoint_eom(
const std::array<double, 4>& x_adjoint, const std::array<double, 2>& cvars, const std::array<double, nargs>& args);

	const StateHistory& _primal_state_history;
	StateHistory& _adjoint_state_history;
	Jacobians _jac ;
	const double _mass;
	// static const int _timesteps_to_update_screen = 10;
	// static bool _updateScreen(int timestep);
};