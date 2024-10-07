#pragma once
#include "EngineMap.h"
#include "Aerodynamics.h"
#include "State.h"
#include "UAV.h"
#include "Poly.h"
#include <stdexcept>
#include <tuple>
#include "Control.h"

#include <cppad/cppad.hpp>
using CppAD::AD; 

class AdjointControl: public Control{
public:
    AdjointControl();
    AdjointControl(Control primal_control): Control(primal_control){}


    
private:
    //function overloading
    AD<double> _getTheta(std::array<, 2> velocity, double t, bool apply_ground_effect, double height);


    Poly<double, Control::_n_coeffs> _adjoint_poly;

};