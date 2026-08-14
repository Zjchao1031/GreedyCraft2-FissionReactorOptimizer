#pragma once

#include "OptimizerTypes.h"

#include <functional>

namespace ncfr::optimizer_detail {

enum class ConductorBridgeCoolingPolicy {
    PerCluster,
    Overall,
};

bool canAttemptConductorBridge(
    const Grid& grid, const FuelSimulation& sim,
    ConductorBridgeCoolingPolicy coolingPolicy =
        ConductorBridgeCoolingPolicy::PerCluster);
std::optional<std::vector<Pos>> shortestConductorPath(
    const Grid& grid, const std::vector<Pos>& starts,
    const StateVector& targetMask, const StateVector* protectedPositions,
    const std::function<bool(const Pos&)>& targetReached,
    const std::atomic_bool* cancelRequested);
int placeConductorsOnPath(Grid& grid, const std::vector<Pos>& path,
                          const StateVector& targetMask,
                          const StateVector& preserveMask);
ConductorBridgeResult connectHeatingClustersWithConductors(
    Grid grid, const FuelSimulation& initialSim,
    const StateVector* protectedPositions,
    const std::atomic_bool* cancelRequested,
    ConductorBridgeCoolingPolicy coolingPolicy =
        ConductorBridgeCoolingPolicy::PerCluster);

#ifndef NDEBUG
void logConductorBridgeCheckpoint(const char* reason, const Grid& grid,
                                   const FuelSimulation& sim,
                                   int clusterCount, int conductorsAdded,
                                   ConductorBridgeCoolingPolicy coolingPolicy =
                                       ConductorBridgeCoolingPolicy::PerCluster);
#endif

} // namespace ncfr::optimizer_detail
