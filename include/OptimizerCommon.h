#pragma once

#include "OptimizerTypes.h"

namespace ncfr::optimizer_detail {

void throwIfCancelled(const std::atomic_bool* cancelRequested);
int dimensionVolume(const Dimension& dim);
int dimensionSpread(const Dimension& dim);
int dimensionSurface(const Dimension& dim);
void validateRequest(const BuildRequest& request);
int requiredSourceCountForFuels(const BuildRequest& request);
std::vector<Dimension> sortedDimensions();
int adjacentCells(const Grid& grid, const Pos& pos);
bool isBetweenFuelCells(const Grid& grid, const Pos& pos, int axis);
RuleContext optimisticRuleContext(const Grid& grid, StateVector& validSinks,
                                  StateVector& functionalCells,
                                  StateVector& activeModerators,
                                  StateVector& activeReflectors,
                                  StateVector& functionalShields,
                                  StateVector& functionalIrradiators);
bool optimisticSinkValidAt(Grid& grid, const Pos& pos,
                           const RuleContext& context);
int manaDustSinkType();
bool isManaDustSink(const Block& block);
std::vector<Pos> manaDustSinkPositions(const Grid& grid);
bool hasEightFunctionalManaDustSinks(const Grid& grid,
                                     const FuelSimulation& sim);
std::optional<Grid> tryPreplaceInsetManaDustFallback(
    Grid grid, std::string* failure = nullptr);
bool isInteriorCorner(const Grid& grid, const Pos& pos);
bool cornerSinkConnectsToInteriorCluster(const Grid& grid,
                                         const Pos& corner);
void removeUnclusteredCornerManaDustSinks(Grid& grid);
void fillSupportBlocks(Grid& grid,
                       const SupportBlockOptions* supportOptions = nullptr,
                       const StateVector* protectedPositions = nullptr);
std::vector<Pos> fuelPositionsInGrid(const Grid& grid);
StateVector protectFuelLineBlocks(const Grid& grid);
bool allSourcesTargetFuel(const Grid& grid);
std::vector<SourcePrimingTarget> sourcePrimingTargets(const Grid& grid);
bool matchesSourcePrimingTargets(
    const Grid& grid,
    const std::vector<SourcePrimingTarget>& expectedTargets);
bool hasRequiredSources(const Grid& grid, const BuildRequest& request);
bool hasNoEmptyInteriorPlane(const Grid& grid);
Grid compactEmptyInteriorPlanes(Grid grid);
bool isSupportMutable(const Block& block);
bool isRequiredSupportBlock(const Grid& grid, const FuelSimulation& sim,
                            int idx);
void pruneInactiveSupport(Grid& grid,
                          const StateVector* protectedPositions = nullptr);
CandidateScore scoreSimulation(
    const Grid& grid, const FuelSimulation& sim,
    CoolingValidationPolicy coolingPolicy =
        CoolingValidationPolicy::PerCluster);
bool betterScore(const CandidateScore& lhs, const CandidateScore& rhs);
std::vector<Pos> improvementPositions(
    const Grid& grid, const FuelSimulation& sim, const ImproveOptions& options,
    const StateVector* protectedPositions = nullptr, bool emptyOnly = false);
Grid improveSupportBlocks(Grid grid, const std::atomic_bool* cancelRequested,
                          const ImproveOptions& options = kDefaultImproveOptions,
                          const SupportBlockOptions* supportOptions = nullptr,
                          const StateVector* protectedPositions = nullptr,
                          bool emptyOnly = false,
                          CoolingValidationPolicy coolingPolicy =
                              CoolingValidationPolicy::PerCluster);
OptimizationResult resultFromSimulation(Grid grid, const BuildRequest& request,
                                        const FuelSimulation& sim);
bool isSearchAccepted(const Grid& grid, const FuelSimulation& sim);
bool isFinalReactorValidInternal(const Grid& grid,
                                 const BuildRequest& request,
                                 const FuelSimulation& sim);
bool isPreCompactRunnable(const FuelSimulation& sim);
FinalizeFailureKind classifyFinalizationFailure(
    const Grid& grid, const FuelSimulation& sim, const BuildRequest& request);
FinalizeFailureKind finalizeFailureFromFuelRelation(
    const FuelRelationPrefilterResult& result);

} // namespace ncfr::optimizer_detail
