#pragma once
#include "EngineMap.h"

class UAV{
public:

    UAV();
    UAV(double surface, double aspect_ratio, double total_mass, double a0, double payload_density); //constructor if not engine given
    UAV(double surface, double aspect_ratio, double total_mass, double a0, double payload_density, const EngineMap& engine);

    static double emptyMass(const UAV& plane);

    void setEngine(const EngineMap& engine);


    //getters
    const double getTotalMass();
    double getSurface();
    double getCl0();
    double getAR();
    double getCd0(){
        return _cd0;
    }
    double getAoaTakeoff(){
        return _aoa_takeoff;
    }

    double getPayloadMass();    //not implemented yet
    double getPayloadVolume();  //not implemented yet

    double setAoaTakeoff(double aoa_takeoff){
        _aoa_takeoff = aoa_takeoff;
    }
private:
    double _surface, _a0, _aspect_ratio, _total_mass;
    double _cd0 = 0.015;
    double _payload_density;
    double _aoa_takeoff = 10*3.14159/180;
    EngineMap _engine;
};