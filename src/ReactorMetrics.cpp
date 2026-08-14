#include "ReactorMetrics.h"

#include "Simulator.h"

#include <algorithm>

namespace ncfr {

int countFunctionalIrradiators(const FuelSimulation& simulation) {
    return static_cast<int>(
        std::count(simulation.functionalIrradiators.begin(),
                   simulation.functionalIrradiators.end(), true));
}

int countUsefulBlocks(const Grid& grid) {
    int count = 0;
    for (const Pos& pos : grid.interiorPositions()) {
        if (isFunctionalInterior(
                grid.at(pos.x, pos.y, pos.z).kind)) {
            ++count;
        }
    }
    return count;
}

double totalIrradiatorFlux(const FuelSimulation& simulation) {
    double total = 0.0;
    for (double flux : simulation.irradiatorFluxByIndex) {
        total += flux;
    }
    return total;
}

void updateResultMetrics(OptimizationResult& result,
                         const FuelSimulation& simulation) {
    result.coolingMargin = overallCoolingMargin(simulation);
    result.usefulBlocks = countUsefulBlocks(result.grid);
    result.disconnectedFunctionalBlocks =
        simulation.disconnectedFunctionalBlocks;
    result.functionalIrradiators =
        countFunctionalIrradiators(simulation);
    result.irradiatorFlux = totalIrradiatorFlux(simulation);
}

} // namespace ncfr
