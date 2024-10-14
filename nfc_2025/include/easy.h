#pragma once

#include "UAV.h"
#include "EngineMap.h"

std::tuple<UAV, EngineMap, std::vector<double>> createAircraftEnigneFromTaskDat();
void saveTaskCns(const UAV &Aircraft);

// void saveTaskResMOO(const Airplane &Aircraft);
// void saveTaskResSOO(Airplane &Aircraft);