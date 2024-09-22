/*  -------------------------------------------------------
 *       CLASS2: EngineMap
 *
 *
 *  -------------------------------------------------------- */

#include "EngineMap.h"

double EngineMap::thrustOfWindspeedCurrent(double &windspeed, double &current)
{
	double x = windspeed;
	double y = current;
	// coefficients from matlab interpolation witn 95% confidence bounds
	double p00, p10, p01, p20, p11, p02;
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
		validPropeller = false;
	}

	return p00 + p10 * x + p01 * y + p20 * pow(x, 2) + p11 * x * y +
		   p02 * pow(y, 2);
}

double EngineMap::powerOfWindspeedCurrent(double &windspeed, double &current)
{
	double x = windspeed;
	double y = current;
	// coefficients from matlab interpolation witn 95% confidence bounds
	double p00, p10, p01, p20, p11, p02;
	p00 = 0;
	p10 = 0;
	p01 = 0;
	p20 = 0;
	p11 = 0;
	p02 = 0;
	if (propeller == P12x6)
	{
		p00 = 0.0101;
		p10 = -0.09204;
		p01 = 12.57;
		p20 = 0.004521;
		p11 = -0.03717;
		p02 = -0.05455;
	}
	else if (propeller == P13x65)
	{
		p00 = 2.333;
		p10 = -0.6026;
		p01 = 12.03;
		p20 = 0.02185;
		p11 = 0.01025;
		p02 = -0.0493;
	}
	else if (propeller == P13x8)
	{
		p00 = 0.973;
		p10 = -0.0912;
		p01 = 11.57;
		p20 = 0.00196;
		p11 = 0.01271;
		p02 = -0.03881;
	}
	else if (propeller == P14X85)
	{
		p00 = -1.808;
		p10 = 0.6065;
		p01 = 11.5;
		p20 = -0.01986;
		p11 = 0.002315;
		p02 = -0.03478;
	}
	else if (propeller == P15x7)
	{
		p00 = 4.403;
		p10 = -1.364;
		p01 = 12.18;
		p20 = 0.05301;
		p11 = -0.0008103;
		p02 = -0.04432;
	}
	else if (propeller == P15x8)
	{
		p00 = -0.4518;
		p10 = 0.6766;
		p01 = 11.41;
		p20 = -0.02981;
		p11 = 0.01263;
		p02 = -0.04048;
	}
	else
	{
		std::cout << "ERROR: prop " << propeller << " is unknown" << std::endl;
	}

	return p00 + p10 * x + p01 * y + p20 * pow(x, 2) + p11 * x * y +
		   p02 * pow(y, 2);
}