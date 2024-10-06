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


    //getters
    double getTotalMass() const;
    double getSurface();
    double getCl0();
    double getAR();
    double getCd0(){
        return _cd0;
    }
    double getAoaTakeoff(){
        return _aoa_takeoff;
    }
    double getWingspan(){
        return sqrt(getAR()*_surface);
    }

    double getPayloadMass();    //not implemented yet
    double getPayloadVolume();  //not implemented yet

    void setAoaTakeoff(double aoa_takeoff){
        _aoa_takeoff = aoa_takeoff;
    }

    double getWheelOffset(){
        return _wheel_offset;
    }
    double getStallAoa(){
        return _aoa_stall_effective + _a0;
    }
    double get_e(){
        return _e;
    }
private:
    double _surface, _a0, _aspect_ratio, _total_mass;
    double _cd0 = 0.015;
    double _payload_density;
    double _aoa_takeoff = 10*3.14159/180;
    double _wheel_offset = 0.1;
    EngineMap _engine;
    double _aoa_stall_effective = 15*3.14159/180;
    const double _e = 0.8; //oswald coeff
};