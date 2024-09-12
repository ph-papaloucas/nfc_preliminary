#include "Airplane.h"
#include "Aerodynamics.h"

#include <iostream>

int main(){
    std::cout <<"Test executable for rclass Airplane\n";
    Airplane vtol(0.5, 9, 10, 0.01);

    
    Aerodynamics aero(vtol);
    std::cout << aero.getCruiseState(20);
    return 0;
}