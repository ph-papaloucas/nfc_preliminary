#include "Airplane.h"
Airplane::Airplane(double surface, double aspect_ratio, double total_mass, double cl0)
    : _surface(surface), _aspect_ratio(aspect_ratio), _total_mass(total_mass), _cl0(cl0){
    // Additional initialization or logic (if needed)
}
Airplane::Airplane(double surface, double aspect_ratio, double total_mass, double cl0, const EngineMap& engine)
    : _surface(surface), _aspect_ratio(aspect_ratio), _total_mass(total_mass), _cl0(cl0), _engine(engine){
    // Additional initialization or logic (if needed)
}


void Airplane::setEngine(const EngineMap& engine){
    _engine = engine;
}

double Airplane::emptyMass(const Airplane& plane){
    //some function that calculates the mass of the airplane according to S, AR, etc

    return plane._total_mass/2;
}

double Airplane::getTotalMass(){
    return _total_mass;
}
double Airplane::getSurface(){
    return _surface;
}
double Airplane::getCl0(){
    return _cl0;
}
double Airplane::getAR(){
    return _aspect_ratio;
}