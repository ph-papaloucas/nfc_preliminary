#include "UAV.h"
UAV::UAV()
: _surface(0), _aspect_ratio(0), _total_mass(0), _a0(0), name("unnamed"){};
UAV::UAV(double surface, double aspect_ratio, double total_mass, double a0)
    : _surface(surface), _aspect_ratio(aspect_ratio), _total_mass(total_mass), _a0(a0), name("unnamed"){
    // Additional initialization or logic (if needed)
}

///////////////// GETTERS /////////////////
double UAV::getTotalMass() const{
    return _total_mass;
}
double UAV::getSurface() const{
    return _surface;
}
double UAV::getCl0() const{
    return 2*M_PI*_a0;
}
double UAV::getAR() const{
    return _aspect_ratio;
}
double UAV::getWheelOffset() const{
    return _wheel_offset;
}
double UAV::getStallAoa() const{
    return _aoa_stall_effective + _a0;
}
double UAV::get_e() const{
    return _e;
}

double UAV::getCd0() const{
    return _cd0;
}
double UAV::getAoaTakeoff() const{
    return _aoa_takeoff;
}
double UAV::getWingspan() const{
    return sqrt(getAR()*_surface);
}


///////////////// SETTERS /////////////////
void UAV::setAoaTakeoff(double aoa_takeoff){
    _aoa_takeoff = aoa_takeoff;
}


void UAV::set_e(double e){
    _e = e;
}