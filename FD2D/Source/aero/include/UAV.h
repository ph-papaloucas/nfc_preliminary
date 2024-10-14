#pragma once
#include "EngineMap.h"
#define _USE_MATH_DEFINES
#include <math.h>


class UAV{
public:
    UAV();
    UAV(double surface, double aspect_ratio, double total_mass, double a0); //constructor if not engine given

    static double emptyMass(const UAV& plane);

    void setEngine(const EngineMap& engine);


    ///////////////// GETTERS /////////////////
    double getTotalMass() const;
    double getSurface() const;
    double getCl0() const;
    double getAR() const;
    double get_e() const;
    double getCd0() const;
    double getAoaTakeoff() const;
    double getWingspan() const;
    double getWheelOffset() const;
    double getStallAoa() const;

    double getPayloadMass() const;    //not implemented yet
    double getPayloadVolume() const;  //not implemented yet

    void setAoaTakeoff(double aoa_takeoff);
    void set_e(double e);


    const std::string name;
    std::vector<double> extra_params;   //this is in Case you need to specify extra parameters for your aircraft, that cannot be generalized
                                        //EXAMPLE: in NFC 2025 there was a payload density parameter needed for the final score of the performance of the aircraft.
                                        //So uppon calling the UAV constructor for nfc 2025, you would add in the next line this:
                                        // extra_params.push_back(payload_density);

private:
    double _surface, _a0, _aspect_ratio, _total_mass;
    double _cd0 = 0.015;
    double _aoa_takeoff = 10*3.14159/180;
    double _wheel_offset = 0.1;
    double _aoa_stall_effective = 15*3.14159/180;
    double _e = 0.8; //oswald coeff
};