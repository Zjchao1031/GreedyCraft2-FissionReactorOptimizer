#pragma once

#include "Data.h"
#include "Grid.h"
#include "StateVector.h"

#include <vector>

namespace ncfr {

struct ClusterStats {
    long long rawHeating = 0;
    long long cooling = 0;
    bool connectedToWall = false;
    int components = 0;
};

struct FuelSimulation {
    bool compatible = false;
    int fuelCells = 0;
    int runningCells = 0;
    long long rawHeating = 0;
    long long cooling = 0;
    long long minClusterMargin = 0;
    int disconnectedFunctionalBlocks = 0;
    std::vector<double> fluxByIndex;
    std::vector<double> irradiatorFluxByIndex;
    std::vector<int> heatLinksByIndex;
    StateVector functionalCells;
    StateVector activeModerators;
    StateVector activeReflectors;
    StateVector functionalShields;
    StateVector functionalIrradiators;
    StateVector validSinks;
    StateVector heatingClusterBlocks;
    std::vector<ClusterStats> clusters;
};

int sourcePrimingTargetIndex(const Grid& grid, const Pos& sourcePos);
FuelSimulation simulateFuel(const Grid& grid, const Fuel& fuel);
FuelSimulation simulateMixedFuel(const Grid& grid);
bool hasSafeFuelFlux(const Grid& grid, const FuelSimulation& sim);
bool isSafeOperatingSimulation(const Grid& grid, const FuelSimulation& sim);

} // namespace ncfr
