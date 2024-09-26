#include "Aerodynamics.h"

Aerodynamics::Aerodynamics(const UAV& uav):_uav(uav){};

double Aerodynamics::_qinf(double velocity){
    return 0.5*1.225*pow(velocity,2);
}

AeroState Aerodynamics::getCruiseState(double velocity){
    double lift = _uav.getTotalMass()*9.81;

    double cl = lift/(_qinf(velocity)*_uav.getSurface());
    double aoa = (cl - _uav.getCl0())/(2*M_PI);
    

    return AeroState(aoa, cl, velocity);
}

std::array<double, 2> Aerodynamics::rotateFromEarth2Bodyframe(std::array<double, 2> vector, double theta){
    return {cos(theta)*vector[0] - sin(theta)*vector[1],
            sin(theta)*vector[0] + cos(theta)*vector[1]};

}
std::array<double, 2> Aerodynamics::rotateFromBody2Earthframe(std::array<double, 2> vector, double theta){
    return {cos(theta)*vector[0] + sin(theta)*vector[1],
            -sin(theta)*vector[0] + cos(theta)*vector[1]};

}
std::array<double,2 > Aerodynamics::rotateFromWind2Earthframe(std::array<double, 2> vector, std::array<double, 2> velocity){
    double theta = atan2(velocity[1], velocity[0]);
    return {cos(theta)*vector[0] + sin(theta)*vector[1],
            -sin(theta)*vector[0] + cos(theta)*vector[1]};
}

double Aerodynamics::rad2deg(double rad){
    return rad*180/M_PI;
}

double Aerodynamics::deg2rad(double deg){
    return deg*M_PI/180;
}


std::array<double, 2> Aerodynamics::getAeroForcesEarthframe(std::array<double, 2> velocity, double theta,  bool apply_ground_effect, double height, double wingspan, double AR){
    double velocity_norm = sqrt( pow(velocity[0],2) + pow(velocity[1],2));
    double qinf = _qinf(velocity_norm);
    double aoa = atan2(velocity[1], velocity[0]) + theta;

    double cl = 2*M_PI*aoa - _uav.getCl0();
    if (apply_ground_effect){
        double cl_increase_factor =
            1 + (288 * pow(height / wingspan, 0.787) *
                    exp(-9.14 * pow(height / wingspan, 0.327)) /
                    pow(AR, 0.882));
        cl = cl_increase_factor * cl;
        #ifdef DEBUG
        if(VERBOSITY_LEVEL >=2)
            std::cout << "Ground effect: " << "height = " << height << "cl_increase_factor = " << cl_increase_factor << std::endl;
        #endif
    }



    double cd = _uav.getCd0() + pow(cl,2)/(2*M_PI*_uav.getAR()*_e);

    double lift = qinf*_uav.getSurface()*cl;
    double drag = qinf*_uav.getSurface()*cd;


    #ifdef DEBUG
        if(VERBOSITY_LEVEL >=3){
            std::cout << "velocity = " << velocity[0] << " " <<velocity[1] << std::endl;
            std::cout << "aoa = " << Aerodynamics::rad2deg(aoa) << std::endl;
            std::cout << "drag = " << drag << "  lift = " << lift <<std::endl;
        }
    #endif
    std::array<double, 2> F = {-drag, lift};


	return    Aerodynamics::rotateFromWind2Earthframe(F, velocity);;

}
std::array<double, 2> Aerodynamics::getAeroForcesEarthframe(std::array<double, 2> velocity, double theta){
    return getAeroForcesEarthframe(velocity, theta, false, 0, 0, 0);

}

// double Aerodynamics::getThetaForTrim(std::array<double, 2> velocity, double thrust){
//     std::cerr << "SOS didnt implement Aerodynamics::getthetaForTrim() yet...\n";
//     std::cerr << "SOS didnt implement Aerodynamics::getthetaForTrim() yet...\n";

//     double iterations = 0;
//     const double tolerance = 0.0001; 
//     double theta_prev = 0;
//     double relax = 0;
//     double gamma = atan2(velocity[1], velocity[0]);

//     // Guesses
//         double theta = 0;
//         std::array<double, 2> u = getAeroForcesEarthframe(velocity, theta);


// 		while ((abs(u[1]) > tolerance) && iterations < max_it)
// 		{ // loop to find aoa to trim
// 			iterations++;
// 			theta_prev = theta;
// 			if (iterations == 200)
// 				relax = 0.15;

// 			aoa = relax * ((((Aircraft.m * G - thrust * sin(aoa + gamma) +
// 							  drag * sin(gamma)) *
// 							 (1 + 2 / Aircraft.AR)) /
// 							(0.5 * DENS * coeff * coeff_ground * Aircraft.S *
// 							 2 * PI * pow(windspeed, 2) * cos(gamma))) +
// 						   Aircraft.a0) +
// 				  (1 - relax) * aoa;

// 			aoa = std::min(aoa - Aircraft.a0, Aircraft.airfoil_stall) +
// 				  Aircraft.a0;

// 			std::tie(cl, cd, coeff_ground) =
// 				getAerodynamicCoeffs(aoa, x[1], Aircraft);
// 			lift = 0.5 * DENS * cl * Aircraft.S * pow(windspeed, 2);
// 			drag = 0.5 * DENS * cd * Aircraft.S * pow(windspeed, 2);
// 			F = rotateForces(thrust, aoa, gamma, lift, drag, roll_resistance,
// 							 Aircraft);
// 			if (iterations > 900)
// 			{
// 				std::cout << aoa << std::endl;
// 			}
// 			if (std::abs(aoa_prev - aoa) < TINY)
// 			{
// 				break;
// 			}
// 		}

// 		if (iterations >= max_it)
// 		{
// 			std::cout << "ERROR: Trim Fz did not converge\n";
// 			std::cout << "\tFz(Should be 0): " << F[1] << std::endl;
// 			std::cout << "\tIterations: " << iterations << std::endl;
// 			std::cout << "\tTolerance: " << tolerance << std::endl;
// 			std::cout << "Max iterations: " << max_it << std::endl;
// 		}
// 		else if ((abs(F[1]) > tolerance))
// 		{
// 			std::cout << "WARNING: Trim Fz could not converge. AoA stabilized "
// 						 "where Fz= "
// 					  << F[1] << std::endl;
// 		}

// 		if (abs(F[1]) < TINY)
// 		{
// 			F[1] = 0;
// 		}
    

//     return 0;
// }