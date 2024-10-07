/*  -------------------------------------------------------
 *       CLASS2: EngineMap
 *
 *
 *  -------------------------------------------------------- */

#include "EngineMap.h"

EngineMap::EngineMap(PROPELLER_enum propeller)
:propeller(propeller){
	p00 = 0;
	p10 = 0;
	p01 = 0;
	p20 = 0;
	p11 = 0;
	p02 = 0;
	if (propeller == P12x6)
	{
		p00 = 0.157;
		p10 = -0.1252;
		p01 = 0.848;
		p20 = -0.004118;
		p11 = -0.0116;
		p02 = -0.007223;
	}
	else if (propeller == P13x65)
	{
		p00 = 0.5261;
		p10 = -0.1959;
		p01 = 0.8247;
		p20 = -0.002268;
		p11 = -0.01013;
		p02 = -0.00622;
	}
	else if (propeller == P13x8)
	{
		p00 = 0.4304;
		p10 = -0.1456;
		p01 = 0.7532;
		p20 = -0.003073;
		p11 = -0.00794;
		p02 = -0.005196;
	}
	else if (propeller == P14X85)
	{
		p00 = 0.714;
		p10 = -0.1716;
		p01 = 0.7453;
		p20 = -0.003225;
		p11 = -0.008028;
		p02 = -0.004771;
	}
	else if (propeller == P15x7)
	{
		p00 = 0.9626;
		p10 = -0.2855;
		p01 = 0.8439;
		p20 = -0.002084;
		p11 = -0.01075;
		p02 = -0.005737;
	}
	else if (propeller == P15x8)
	{
		p00 = 1.127;
		p10 = -0.2072;
		p01 = 0.757;
		p20 = -0.003704;
		p11 = -0.00825;
		p02 = -0.004828;
	}
	else
	{
		std::cout << "ERROR: prop " << propeller << " is unknown" << std::endl;
		//validPropeller = false;
	}
}

double EngineMap::thrustOfWindspeedCurrent(const double &windspeed, const double &current) const
{
	double x = windspeed;
	double y = current;
	// coefficients from matlab interpolation witn 95% confidence bounds
	return p00 + p10 * x + p01 * y + p20 * pow(x, 2) + p11 * x * y +
		   p02 * pow(y, 2);
}

CppAD::AD<double> EngineMap::thrustOfWindspeedCurrent(CppAD::AD<double> windspeed, double current) const
{
	CppAD::AD<double> x = windspeed;
	double y = current;
	// coefficients from matlab interpolation witn 95% confidence bounds
	return p00 + p10 * x + p01 * y + p20 * pow(x, 2) + p11 * x * y +
		   p02 * pow(y, 2);
}

double EngineMap::powerOfWindspeedCurrent(double &windspeed, double &current)
{
	double x = windspeed;
	double y = current;
	// coefficients from matlab interpolation witn 95% confidence bounds
	double c00, c10, c01, c20, c11, c02;
	c00 = 0;
	c10 = 0;
	c01 = 0;
	c20 = 0;
	c11 = 0;
	c02 = 0;
	if (propeller == P12x6)
	{
		c00 = 0.0101;
		c10 = -0.09204;
		c01 = 12.57;
		c20 = 0.004521;
		c11 = -0.03717;
		c02 = -0.05455;
	}
	else if (propeller == P13x65)
	{
		c00 = 2.333;
		c10 = -0.6026;
		c01 = 12.03;
		c20 = 0.02185;
		c11 = 0.01025;
		c02 = -0.0493;
	}
	else if (propeller == P13x8)
	{
		c00 = 0.973;
		c10 = -0.0912;
		c01 = 11.57;
		c20 = 0.00196;
		c11 = 0.01271;
		c02 = -0.03881;
	}
	else if (propeller == P14X85)
	{
		c00 = -1.808;
		c10 = 0.6065;
		c01 = 11.5;
		c20 = -0.01986;
		c11 = 0.002315;
		c02 = -0.03478;
	}
	else if (propeller == P15x7)
	{
		c00 = 4.403;
		c10 = -1.364;
		c01 = 12.18;
		c20 = 0.05301;
		c11 = -0.0008103;
		c02 = -0.04432;
	}
	else if (propeller == P15x8)
	{
		c00 = -0.4518;
		c10 = 0.6766;
		c01 = 11.41;
		c20 = -0.02981;
		c11 = 0.01263;
		c02 = -0.04048;
	}
	else
	{
		std::cout << "ERROR: prop " << propeller << " is unknown" << std::endl;
	}

	return c00 + c10 * x + c01 * y + c20 * pow(x, 2) + c11 * x * y +
		   c02 * pow(y, 2);
}