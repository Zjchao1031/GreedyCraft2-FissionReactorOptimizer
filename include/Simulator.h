#pragma once

#include "Data.h"
#include "Grid.h"
#include "StateVector.h"

#include <vector>

namespace ncfr {

struct ClusterStats {
    long long rawHeating = 0;
    long long cooling = 0;
    int components = 0;
};

struct SimulationOptions {
    const StateVector* forcedValidSinks = nullptr;
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

struct WallConnectionResult {
    int heatingClusters = 0;
    int disconnectedHeatingClusters = 0;

    bool allConnected() const { return heatingClusters > 0 && disconnectedHeatingClusters == 0; }
};

int sourcePrimingTargetIndex(const Grid& grid, const Pos& sourcePos);
FuelSimulation simulateFuel(const Grid& grid, const Fuel& fuel, const SimulationOptions& options = {});
FuelSimulation simulateMixedFuel(const Grid& grid, const SimulationOptions& options = {});
WallConnectionResult evaluateHeatingClusterWallConnections(const Grid& grid, const FuelSimulation& sim);
bool hasSafeFuelFlux(const Grid& grid, const FuelSimulation& sim);
bool hasInvalidSinks(const Grid& grid, const FuelSimulation& sim);
long long overallCoolingMargin(const FuelSimulation& sim);
bool hasOverallCoolingMargin(const FuelSimulation& sim);
bool isOverallCoolingOperatingSimulation(const Grid& grid, const FuelSimulation& sim);
bool isSearchOperatingSimulation(const Grid& grid, const FuelSimulation& sim);
bool isSafeOperatingSimulation(const Grid& grid, const FuelSimulation& sim);

} // namespace ncfr
