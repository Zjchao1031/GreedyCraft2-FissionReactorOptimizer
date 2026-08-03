#pragma once

#include "Optimizer.h"

namespace ncfr {

struct FuelSimulation;

int countFunctionalIrradiators(const FuelSimulation& simulation);
int countUsefulBlocks(const Grid& grid);
double totalIrradiatorFlux(const FuelSimulation& simulation);
void updateResultMetrics(OptimizationResult& result,
                         const FuelSimulation& simulation);

} // namespace ncfr
