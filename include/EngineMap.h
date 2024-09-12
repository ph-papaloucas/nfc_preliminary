#pragma once 

#include "cmath"
#include <iostream>
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

    EngineMap():propeller(PROPELLER_enum::UNDEF1){};            //defaul constructor
    EngineMap(PROPELLER_enum propeller):propeller(propeller){};
	
    int propeller;
	bool validPropeller = true;


	double thrustOfWindspeedCurrent(double &windspeed, double &current);
	double powerOfWindspeedCurrent(double &windspeed, double &current);
};