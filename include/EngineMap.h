#pragma once 

#include "cmath"
#include <iostream>
#include <array>
class EngineMap
{
  public:
	// Propeller choices
	enum PROPELLER_enum
	{
		P12x6,
		P13x65,
		P13x8,
		P14X85,
		P15x7,
		P15x8,
		UNDEF1,
		UNDEF2
	};

    EngineMap()
	:propeller(PROPELLER_enum::UNDEF1), p00(0e0), p10(0e0), p20(0e0), p01(0e0), p02(0e0), p11(0e0){};            //defaul constructor
    EngineMap(PROPELLER_enum propeller);
    int propeller;
	bool validPropeller = true;


	double thrustOfWindspeedCurrent(const double &windspeed, const double &current) const;
	double powerOfWindspeedCurrent(double &windspeed, double &current);

	std::array<double, 6> getEngineCoeffs(){
		return std::array<double, 6>{p00, p10, p20, p01, p02, p11};
	}

private:
	double p00, p10, p20, p01, p02, p11;
};