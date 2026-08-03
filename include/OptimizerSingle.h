#pragma once

#include "OptimizerTypes.h"

namespace ncfr::optimizer_detail {

FinalizeResult acceptedResultFromImprovedGrid(
    Grid improved, const FuelSimulation& sim, const BuildRequest& request,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines,
    const char* compactFailureReasonPrefix, bool keepConductors = false);
FinalizeResult tryFinalizeDirectionalCandidate(
    Grid grid, const BuildRequest& request,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines,
    const StateVector* protectedPositions,
    const std::atomic_bool* cancelRequested,
    std::optional<MergeableSingleFuelLayout>* mergeableBest = nullptr,
    const MergeableSingleFuelSearchGoal* mergeableGoal = nullptr,
    bool* mergeableGoalReached = nullptr);
OptimizationResult optimizeSingleFuelDirectionalLayout(
    const BuildRequest& request,
    const std::vector<std::vector<int>>& sourceCombos,
    const std::atomic_bool* cancelRequested);
BuildRequest singleFuelRequestForSlot(const BuildRequest& request, int slot);
OptimizationResult optimizeSingleFuelForSlot(const BuildRequest& request, int slot,
                                             const std::atomic_bool* cancelRequested);
MergeableSingleFuelLayout optimizeMergeableSingleFuelForSlot(
    const BuildRequest& request, int slot,
    const MergeableSingleFuelSearchGoal& goal,
    const std::atomic_bool* cancelRequested);
bool heatPriorityLess(int lhsSlot, int rhsSlot, const BuildRequest& request);

} // namespace ncfr::optimizer_detail
