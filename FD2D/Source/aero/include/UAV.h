#pragma once
#include "EngineMap.h"
#define _USE_MATH_DEFINES
#include <math.h>


class UAV{
public:

    UAV();
    UAV(double surface, double aspect_ratio, double total_mass, double a0, double payload_density); //constructor if not engine given
    UAV(double surface, double aspect_ratio, double total_mass, double a0, double payload_density, const EngineMap& engine);

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
private:
    double _surface, _a0, _aspect_ratio, _total_mass;
    double _cd0 = 0.015;
    double _payload_density;
    double _aoa_takeoff = 10*3.14159/180;
    double _wheel_offset = 0.1;
    EngineMap _engine;
    double _aoa_stall_effective = 15*3.14159/180;
    double _e = 0.8; //oswald coeff
};