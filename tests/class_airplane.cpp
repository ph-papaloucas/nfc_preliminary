#include "Airplane.h"
#include "EngineMap.h"
#include <iostream>

int main(){
    std::cout <<"Test executable for rclass Airplane\n";

    Airplane vtol(0.5, 9, 10, 0.01);

    std::cout << "function emptyMass():" << vtol.emptyMass(vtol) << std::endl;
    return 0;
}