#include "UAV.h"
UAV::UAV()
: _surface(0), _aspect_ratio(0), _total_mass(0), _a0(0), _payload_density(0){};
UAV::UAV(double surface, double aspect_ratio, double total_mass, double a0, double payload_density)
    : _surface(surface), _aspect_ratio(aspect_ratio), _total_mass(total_mass), _a0(a0), _payload_density(payload_density){
    // Additional initialization or logic (if needed)
}
UAV::UAV(double surface, double aspect_ratio, double total_mass, double a0, double payload_density, const EngineMap& engine)
    : _surface(surface), _aspect_ratio(aspect_ratio), _total_mass(total_mass), _a0(a0), _payload_density(payload_density), _engine(engine){
    // Additional initialization or logic (if needed)
}


void UAV::setEngine(const EngineMap& engine){
    _engine = engine;
}

double UAV::emptyMass(const UAV& plane){
    //some function that calculates the mass of the UAV according to S, AR, etc

    return plane._total_mass/2;
}

double UAV::getTotalMass() const{
    return _total_mass;
}
double UAV::getSurface(){
    return _surface;
}
double UAV::getCl0(){
    return 2*M_PI*_a0;
}
double UAV::getAR(){
    return _aspect_ratio;
}