#include "easy.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <tuple>

std::tuple<UAV, EngineMap, std::vector<double>> createAircraftEnigneFromTaskDat()
{
	/// Reads task.dat (output tou easy), and creates the Aircraft.
	const size_t num_of_inputs = 7;
	// Variables to be read from the file
	double WCL, AR, a0, aoa_takeoff;
	int prop;
	double mass_calc_parameter;
	double climb_angle;

	// Initialize the variables to zero
	WCL = 0;
	AR = 0;
	a0 = 0;
	aoa_takeoff = 0;
	prop = 0;
	mass_calc_parameter = -100;
	climb_angle = 0;

	// Open the file
	std::array<double, num_of_inputs + 1> in = {}; 
	std::fstream TaskFile;
	TaskFile.open("task.dat", std::ios::in);
	if (!TaskFile)
	{
		std::cout << "Error: Unable to open task.dat for reading." << std::endl;
		std::cerr << "Error: Unable to open task.dat for reading." << std::endl;
		throw std::runtime_error("Error: Unable to open task.dat for reading.");
	}
	std::string line;
	size_t counter = 0;
	while (std::getline(TaskFile, line)){
		std::istringstream iss(line);
		iss >> in[counter];
		std::cout << line << "->" << in[counter] << "\n";
		counter++;
	}
	TaskFile.close();

	WCL = in[1];
	AR = in[2];
	a0 = in[3];
	aoa_takeoff = in[4];
	prop = in[5];
	mass_calc_parameter = in[6];
	climb_angle = in[7];

#ifdef DEBUG

    std::cout << "\n\nInput of task.dat: \n";
    std::cout << "\tWCL     " << WCL << "\n\tAR   " << AR << "\n\tprop    "
                << prop;
    std::cout << "\n\tnballs_half " << nballs_half << "\n\ta0       " << a0
                << "\n\taoa_takeoff " << aoa_takeoff;

    std::cout << "\n\tx_hover " << x_hover << "\n\th1_target " << h1_target
                << "\n\th2_target " << h2_target << "\n\th3_target "
                << h3_target << "\n\th4_target " << h4_target << "\n\tdt2 "
                << dt2 << "\n\tdt4 " << dt4 << "\n\tI_efficiency "
                << I_efficiency << "\n\tflaps retract height "
                << flaps_retract_h << "\n\tmass calc parameter "
                << mass_calc_parameter << "\n\n";
#endif

    double S =0.5;
    double mass =3;
    UAV Aircraft(S, AR, mass, a0);
	EngineMap engine(EngineMap::P12x6);
	std::cout << Aircraft.name << " created\n";
	return std::make_tuple(Aircraft, engine, std::vector<double>{climb_angle});
}


void saveTaskCns(const UAV &Aircraft)
{
	std::ofstream fout_cns("task.cns");
	if (fout_cns.is_open())
	{
		fout_cns.close();
	}
	else
	{
		// Handle the case when the file cannot be opened
		std::cerr << "Error: Unable to open error.txt for writing."
				  << std::endl;
	}
	std::ofstream fout_cnsnames("tasknames.cns");
	if (fout_cnsnames.is_open())
	{
		fout_cnsnames.close();
	}
	else
	{
		// Handle the case when the file cannot be opened
		std::cerr << "Error: Unable to open error.txt for writing."
				  << std::endl;
	}
}