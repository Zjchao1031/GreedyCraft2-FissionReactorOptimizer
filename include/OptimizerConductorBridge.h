#pragma once

#include "OptimizerTypes.h"

#include <functional>

namespace ncfr::optimizer_detail {

bool canAttemptConductorBridge(const Grid& grid, const FuelSimulation& sim);
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
    const std::atomic_bool* cancelRequested);

#ifndef NDEBUG
void logConductorBridgeCheckpoint(const char* reason, const Grid& grid,
                                  const FuelSimulation& sim,
                                  int clusterCount, int conductorsAdded);
#endif

} // namespace ncfr::optimizer_detail
