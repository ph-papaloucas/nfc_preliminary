#pragma once
#include "EngineMap.h"

class Airplane{
public:

    Airplane(double surface, double aspect_ratio, double total_mass, double cl0); //constructor if not engine given
    Airplane(double surface, double aspect_ratio, double total_mass, double cl0, const EngineMap& engine);

    static double emptyMass(const Airplane& plane);

    void setEngine(const EngineMap& engine);


    //getters
    double getTotalMass();
    double getSurface();
    double getCl0();
    double getAR();
private:
    double _surface, _cl0, _aspect_ratio, _total_mass;

    EngineMap _engine;
};