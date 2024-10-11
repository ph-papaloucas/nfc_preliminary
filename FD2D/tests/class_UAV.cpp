#include "UAV.h"
#include "EngineMap.h"
#include <iostream>

int main(){
    std::cout <<"Test executable for rclass UAV\n";

    UAV vtol(0.5, 9, 10, 0.01, 1);

    std::cout << "function emptyMass():" << vtol.emptyMass(vtol) << std::endl;
    return 0;
}