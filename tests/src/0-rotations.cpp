#include "Aerodynamics.h"
#include <array>

void print(std::array<double, 2> x){
    std::cout << "(" << x[0] <<  ", " << x[1] << " )" ;
}
int main(){
    double Lift = 50;
    double Drag = 5;

    std::array<double, 2> v = {10, 2};

    std::array<double, 2> Fw;

    Fw = {Drag, Lift};
    std::array<double, 2> F = Aerodynamics::rotateFromWind2Earthframe(Fw, v);

    print(Fw) ;
    print(F);
    std::cout << std::endl;
    return 0;
}

