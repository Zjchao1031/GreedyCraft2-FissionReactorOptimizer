#include "OptimizerSingle.h"

#include "OptimizerCommon.h"
#include "OptimizerConductorBridge.h"
#include "OptimizerCooling.h"
#include "OptimizerDiagnostics.h"
#include "OptimizerDirectional.h"
#include "OptimizerSpecialCooling.h"

#include "FuelPlacementPrefilter.h"
#include "FuelSpecialCases.h"
#include "Perf.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
namespace ncfr::optimizer_detail {

constexpr double kFluxEpsilon = 1e-9;

struct SingleFuelSkeletonSpec {
    std::vector<FuelLineSpec> lines;
    double estimatedFlux = 0.0;
};

struct BuiltSingleFuelSkeleton {
    Grid grid;
    StateVector protectedPositions;
    std::vector<FuelLineSpec> fuelLines;
};

struct SingleFuelSkeletonSearch {
    const Fuel* fuel = nullptr;
    const BuildRequest* request = nullptr;
    const Dimension* dim = nullptr;
    const std::vector<int>* sourceDirections = nullptr;
    const std::vector<std::vector<FuelLineSpec>>* perDirectionOptions = nullptr;
    const std::atomic_bool* cancelRequested = nullptr;
    std::optional<MergeableSingleFuelLayout>* mergeableBest = nullptr;
    const MergeableSingleFuelSearchGoal* mergeableGoal = nullptr;
    bool* mergeableGoalReached = nullptr;
    int targetLineCount = 1;
};

std::vector<int> lineDirections(const std::vector<FuelLineSpec>& lines) {
    std::vector<int> directions;
    directions.reserve(lines.size());
    for (const FuelLineSpec& line : lines) {
        directions.push_back(line.direction);
    }
    std::sort(directions.begin(), directions.end());
    return directions;
}

std::vector<int> lineDirections(const SingleFuelSkeletonSpec& spec) {
    return lineDirections(spec.lines);
}

std::optional<BuiltSingleFuelSkeleton> buildSingleFuelSkeleton(const Dimension& dim,
                                                               const BuildRequest& request,
                                                               const std::vector<int>& sourceDirections,
                                                               const SingleFuelSkeletonSpec& spec) {
    Grid grid = makeShell(dim.a, dim.b, dim.c);
    StateVector protectedPositions(static_cast<size_t>(grid.volume()), false);
    const Pos fuelPos{(dim.a + 1) / 2, (dim.b + 1) / 2, (dim.c + 1) / 2};
    grid.at(fuelPos.x, fuelPos.y, fuelPos.z) = {BlockKind::FuelCell, request.fuelIndices.front()};
    markProtected(protectedPositions, grid, fuelPos);

    for (const FuelLineSpec& line : spec.lines) {
        if (!fuelLineWithinReflectorReach(line)) {
            return std::nullopt;
        }
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(line.direction));
        for (int distance = 1; distance <= line.moderatorCount; ++distance) {
            const Pos moderatorPos = offset(fuelPos, dir, distance);
            if (!grid.isInterior(moderatorPos.x, moderatorPos.y, moderatorPos.z)) {
                return std::nullopt;
            }
            Block& block = grid.at(moderatorPos.x, moderatorPos.y, moderatorPos.z);
            if (block.kind != BlockKind::Empty) {
                return std::nullopt;
            }
            block = {BlockKind::Moderator, line.moderatorType};
            markProtected(protectedPositions, grid, moderatorPos);
        }

        const Pos reflectorPos = offset(fuelPos, dir, line.moderatorCount + 1);
        if (!grid.isInterior(reflectorPos.x, reflectorPos.y, reflectorPos.z)) {
            return std::nullopt;
        }
        Block& reflector = grid.at(reflectorPos.x, reflectorPos.y, reflectorPos.z);
        if (reflector.kind != BlockKind::Empty) {
            return std::nullopt;
        }
        reflector = {BlockKind::Reflector, line.reflectorType};
        markProtected(protectedPositions, grid, reflectorPos);
    }

    if (!placeDirectionalSources(grid, request, fuelPos, sourceDirections)) {
        return std::nullopt;
    }
    return BuiltSingleFuelSkeleton{std::move(grid), std::move(protectedPositions), spec.lines};
}

std::optional<OptimizationResult> trySingleFuelSkeletonSpec(const SingleFuelSkeletonSearch& search,
                                                            const SingleFuelSkeletonSpec& spec) {
    const std::vector<int> reflectorDirections = lineDirections(spec);
    throwIfCancelled(search.cancelRequested);
#ifndef NDEBUG
    {
        const std::string detail =
            directionalCandidateDetail("start", *search.dim, *search.sourceDirections, reflectorDirections);
        NCFR_PERF_CHECKPOINT("candidate.directional", detail.c_str());
    }
#endif
    std::optional<BuiltSingleFuelSkeleton> candidate;
    {
        NCFR_PERF_SCOPE(candidateGenerationNs);
        candidate = buildSingleFuelSkeleton(*search.dim, *search.request, *search.sourceDirections, spec);
    }
    if (!candidate.has_value()) {
#ifndef NDEBUG
        const std::string detail =
            directionalCandidateDetail("skeletonRejected", *search.dim, *search.sourceDirections, reflectorDirections);
        NCFR_PERF_CHECKPOINT("candidate.directional", detail.c_str());
#endif
        return std::nullopt;
    }
    const FuelRelationPrefilterResult relation = prefilterFuelRelations(candidate->grid, *search.request);
    if (!relation.accepted) {
#ifndef NDEBUG
        std::ostringstream detail;
        detail << directionalCandidateDetail("fuelRelationRejected", *search.dim, *search.sourceDirections,
                                             reflectorDirections)
               << " " << fuelRelationDetail("prefilter", relation, *search.request);
        const std::string checkpoint = detail.str();
        NCFR_PERF_CHECKPOINT("candidate.directional", checkpoint.c_str());
#endif
        return std::nullopt;
    }
    NCFR_PERF_COUNT(candidateCount);
    NCFR_PERF_COUNT(candidateEvaluations);
    NCFR_PERF_SCOPE(candidateEvaluationNs);
    throwIfCancelled(search.cancelRequested);
    FinalizeResult result = tryFinalizeDirectionalCandidate(
        std::move(candidate->grid), *search.request, *search.sourceDirections, candidate->fuelLines,
        &candidate->protectedPositions, search.cancelRequested,
        search.mergeableBest, search.mergeableGoal,
        search.mergeableGoalReached);
    if (result.result.has_value()) {
        NCFR_PERF_COUNT(bestUpdates);
        return std::move(*result.result);
    }
    return std::nullopt;
}

std::optional<OptimizationResult> enumerateSingleFuelSkeletonSpecs(const SingleFuelSkeletonSearch& search,
                                                                   int startDirection,
                                                                   SingleFuelSkeletonSpec& current) {
    throwIfCancelled(search.cancelRequested);
    const Fuel& fuel = *search.fuel;
    if (!current.lines.empty() &&
        current.estimatedFlux + kFluxEpsilon >= fuel.criticality &&
        current.estimatedFlux <= 2.0 * fuel.criticality + kFluxEpsilon) {
        if (static_cast<int>(current.lines.size()) != search.targetLineCount) {
            return std::nullopt;
        }
        return trySingleFuelSkeletonSpec(search, current);
    }
    if (static_cast<int>(current.lines.size()) >= search.targetLineCount) {
        return std::nullopt;
    }
    if (current.estimatedFlux > 2.0 * fuel.criticality + kFluxEpsilon) {
        return std::nullopt;
    }

    for (int direction = startDirection;
         direction < static_cast<int>(search.perDirectionOptions->size());
         ++direction) {
        throwIfCancelled(search.cancelRequested);
        for (const FuelLineSpec& option : search.perDirectionOptions->at(static_cast<size_t>(direction))) {
            throwIfCancelled(search.cancelRequested);
            if (current.estimatedFlux + option.estimatedFlux > 2.0 * fuel.criticality + kFluxEpsilon) {
                continue;
            }
            current.lines.push_back(option);
            current.estimatedFlux += option.estimatedFlux;
            std::optional<OptimizationResult> result =
                enumerateSingleFuelSkeletonSpecs(
                    search, direction + 1, current);
            if (result.has_value() ||
                (search.mergeableGoalReached != nullptr &&
                 *search.mergeableGoalReached)) {
                return result;
            }
            current.estimatedFlux -= option.estimatedFlux;
            current.lines.pop_back();
        }
    }
    return std::nullopt;
}

std::optional<OptimizationResult> searchSingleFuelSkeletonSpecs(const SingleFuelSkeletonSearch& search) {
    std::vector<std::vector<FuelLineSpec>> perDirectionOptions;
    perDirectionOptions.reserve(kSourceDirections.size());
    for (int direction = 0; direction < static_cast<int>(kSourceDirections.size()); ++direction) {
        throwIfCancelled(search.cancelRequested);
        perDirectionOptions.push_back(singleFuelLineOptions(*search.fuel, *search.request,
                                                            *search.sourceDirections, direction));
    }

    SingleFuelSkeletonSearch localSearch = search;
    localSearch.perDirectionOptions = &perDirectionOptions;
    for (int lineCount = 1; lineCount <= static_cast<int>(kSourceDirections.size()); ++lineCount) {
        throwIfCancelled(search.cancelRequested);
        localSearch.targetLineCount = lineCount;
        SingleFuelSkeletonSpec current;
        std::optional<OptimizationResult> result =
            enumerateSingleFuelSkeletonSpecs(localSearch, 0, current);
        if (result.has_value() ||
            (localSearch.mergeableGoalReached != nullptr &&
             *localSearch.mergeableGoalReached)) {
            return result;
        }
    }
    return std::nullopt;
}


int singleFuelHeatingClusterCount(const FuelSimulation& sim) {
    return static_cast<int>(std::count_if(
        sim.clusters.begin(), sim.clusters.end(),
        [](const ClusterStats& cluster) {
            return cluster.rawHeating > 0;
        }));
}

bool hasValidHeatingSink(const Grid& grid, const FuelSimulation& sim) {
    for (const Pos& pos : grid.interiorPositions()) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        if (grid.atIndex(idx).kind == BlockKind::Sink &&
            sim.validSinks.at(static_cast<size_t>(idx)) &&
            sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            return true;
        }
    }
    return false;
}

bool betterMergeableSingleFuelLayout(
    const MergeableSingleFuelLayout& candidate,
    const MergeableSingleFuelLayout& current) {
    if (candidate.sim.minClusterMargin != current.sim.minClusterMargin) {
        return candidate.sim.minClusterMargin > current.sim.minClusterMargin;
    }
    if (candidate.sim.cooling != current.sim.cooling) {
        return candidate.sim.cooling > current.sim.cooling;
    }
    const int candidateVolume =
        candidate.grid.internalA() * candidate.grid.internalB() *
        candidate.grid.internalC();
    const int currentVolume =
        current.grid.internalA() * current.grid.internalB() *
        current.grid.internalC();
    if (candidateVolume != currentVolume) {
        return candidateVolume < currentVolume;
    }
    return countUsefulBlocks(candidate.grid) <
           countUsefulBlocks(current.grid);
}

bool mergeableSingleFuelGoalReached(
    const FuelSimulation& sim,
    const MergeableSingleFuelSearchGoal& goal) {
    if (sim.cooling >= goal.minimumCooling) {
        return true;
    }
    return goal.allowCombinedBalance &&
           goal.pairedCooling + sim.cooling >=
               goal.pairedRawHeating + sim.rawHeating;
}

void considerMergeableSingleFuelLayout(
    const Grid& grid, const BuildRequest& request,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines,
    std::optional<MergeableSingleFuelLayout>* mergeableBest,
    const MergeableSingleFuelSearchGoal* mergeableGoal,
    bool* mergeableGoalReached) {
    if (mergeableBest == nullptr) {
        return;
    }

    FuelSimulation sim = simulateMixedFuel(grid);
    if (!isPreCompactRunnable(sim) ||
        !hasSafeFuelFlux(grid, sim) ||
        sim.disconnectedFunctionalBlocks != 0 ||
        hasInvalidSinks(grid, sim) ||
        singleFuelHeatingClusterCount(sim) != 1 ||
        !hasValidHeatingSink(grid, sim) ||
        !hasRequiredSources(grid, request)) {
        return;
    }

    if (mergeableGoal != nullptr &&
        !mergeableSingleFuelGoalReached(sim, *mergeableGoal)) {
        return;
    }

    MergeableSingleFuelLayout candidate{
        grid,
        std::move(sim),
        sourceDirections,
        fuelLines,
    };
    if (!mergeableBest->has_value() ||
        betterMergeableSingleFuelLayout(candidate, **mergeableBest)) {
        *mergeableBest = std::move(candidate);
    }
    if (mergeableGoalReached != nullptr) {
        *mergeableGoalReached = true;
    }
}

FinalizeResult acceptedResultFromImprovedGrid(Grid improved, const FuelSimulation& sim,
                                              const BuildRequest& request,
                                              const std::vector<int>& sourceDirections,
                                               const std::vector<FuelLineSpec>& fuelLines,
                                               const char* compactFailureReasonPrefix,
                                               bool keepConductors) {
    (void)sim;
    (void)compactFailureReasonPrefix;
    const std::vector<int> reflectorDirections = lineDirections(fuelLines);
    std::optional<Grid> finalCompacted =
        compactInteriorPlanesPreservingSources(improved, request, sourceDirections, fuelLines, 0, keepConductors);
    bool finalHasNoEmptyPlane = false;
    if (finalCompacted.has_value()) {
        finalHasNoEmptyPlane = hasNoEmptyInteriorPlane(*finalCompacted);
    }
    if (!finalCompacted.has_value() || !finalHasNoEmptyPlane) {
#ifndef NDEBUG
        const char* reason = compactFailureReasonPrefix;
        if (!finalCompacted.has_value()) {
            reason = "finalCompactPreservingSourcesFailed";
        } else if (!finalHasNoEmptyPlane) {
            reason = "finalEmptyInteriorPlane";
        }
        const Grid& detailGrid = finalCompacted.has_value() ? *finalCompacted : improved;
        const std::string detail = directionalGridDetail(reason, detailGrid, nullptr, request, sourceDirections,
                                                         reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, FinalizeFailureKind::Structural};
    }
    FuelSimulation finalSim = simulateMixedFuel(*finalCompacted);
    const WallConnectionResult wall =
        evaluateHeatingClusterWallConnections(*finalCompacted, finalSim);
    if (!isSearchAccepted(*finalCompacted, finalSim) || !wall.allConnected()) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("finalNotAccepted", *finalCompacted, &finalSim, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
        std::ostringstream wallDetail;
        wallDetail << "grid=" << gridInteriorLabel(*finalCompacted)
                   << " heatingClusters=" << wall.heatingClusters
                   << " wallDisconnected=" << wall.disconnectedHeatingClusters;
        NCFR_PERF_CHECKPOINT("wallConnection.final", wallDetail.str().c_str());
#endif
        return {
            std::nullopt,
            isSearchAccepted(*finalCompacted, finalSim)
                ? FinalizeFailureKind::WallDisconnected
                : classifyFinalizationFailure(*finalCompacted, finalSim, request)};
    }
#ifndef NDEBUG
    const std::string detail = directionalGridDetail("accepted", *finalCompacted, &finalSim, request,
                                                     sourceDirections, reflectorDirections);
    logFinalizeCheckpoint("finalize.accept", detail, 0, kDefaultImproveOptions);
#endif
    OptimizationResult result =
        resultFromSimulation(std::move(*finalCompacted), request, finalSim);
    return {std::move(result), FinalizeFailureKind::None};
}


std::optional<FinalizeResult> tryConductorBridgeFinalization(
    const Grid& grid, const FuelSimulation& sim, const BuildRequest& request,
    const std::vector<int>& sourceDirections, const std::vector<FuelLineSpec>& fuelLines,
    const StateVector* protectedPositions, const std::atomic_bool* cancelRequested) {
    if (!canAttemptConductorBridge(grid, sim)) {
        return std::nullopt;
    }
    ConductorBridgeResult bridge = connectHeatingClustersWithConductors(grid, sim, protectedPositions, cancelRequested);
#ifndef NDEBUG
    logConductorBridgeCheckpoint(bridge.reason.c_str(), bridge.grid, bridge.sim, bridge.clusterCount,
                                 bridge.conductorsAdded);
#endif
    if (!bridge.attempted || !bridge.success) {
        return std::nullopt;
    }
    pruneInactiveSupport(bridge.grid, protectedPositions);
    bridge.sim = simulateMixedFuel(bridge.grid);
    if (!isSearchAccepted(bridge.grid, bridge.sim)) {
#ifndef NDEBUG
        logConductorBridgeCheckpoint("prunedNotAccepted", bridge.grid, bridge.sim, bridge.clusterCount,
                                     bridge.conductorsAdded);
#endif
        return std::nullopt;
    }
    FinalizeResult finalResult = acceptedResultFromImprovedGrid(
        std::move(bridge.grid), bridge.sim, request, sourceDirections, fuelLines,
        "conductorBridgeCompactValidationFailed");
    if (!finalResult.result.has_value()) {
        return std::nullopt;
    }
    return finalResult;
}

FinalizeResult tryFinalizeDirectionalCandidate(Grid grid, const BuildRequest& request,
                                               const std::vector<int>& sourceDirections,
                                               const std::vector<FuelLineSpec>& fuelLines,
                                               const StateVector* protectedPositions,
                                               const std::atomic_bool* cancelRequested,
                                               std::optional<MergeableSingleFuelLayout>* mergeableBest,
                                               const MergeableSingleFuelSearchGoal* mergeableGoal,
                                               bool* mergeableGoalReached) {
    const std::vector<int> reflectorDirections = lineDirections(fuelLines);
    const bool mergeableTargetMode = mergeableGoal != nullptr;
    const SupportBlockOptions supportOptions{
        request.selectedModeratorTypeIndices,
        request.selectedReflectorTypeIndices,
    };
    NCFR_PERF_COUNT(finalizeCandidateCalls);
    NCFR_PERF_SCOPE(finalizeCandidateNs);
    const FuelRelationPrefilterResult relation = prefilterFuelRelations(grid, request);
    if (!relation.accepted) {
#ifndef NDEBUG
        std::ostringstream detail;
        detail << directionalGridDetail("fuelRelationRejected", grid, nullptr, request, sourceDirections,
                                        reflectorDirections)
               << " " << fuelRelationDetail("prefilter", relation, request);
        const std::string checkpoint = detail.str();
        logFinalizeCheckpoint("finalize.reject", checkpoint, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, finalizeFailureFromFuelRelation(relation)};
    }
    pruneInactiveSupport(grid, protectedPositions);
    FuelSimulation sim = simulateMixedFuel(grid);
#ifndef NDEBUG
    {
        std::ostringstream os;
        os << "mode=single grid=" << gridInteriorLabel(grid)
           << " compatible=" << (sim.compatible ? 1 : 0)
           << " rawHeating=" << sim.rawHeating
           << " cooling=" << sim.cooling
           << " minMargin=" << sim.minClusterMargin
           << " disconnected=" << sim.disconnectedFunctionalBlocks;
        NCFR_PERF_CHECKPOINT("simulation.search", os.str().c_str());
    }
#endif
    if (!isPreCompactRunnable(sim)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("preOptimizeNotRunnable", grid, &sim, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, classifyFinalizationFailure(grid, sim, request)};
    }
    considerMergeableSingleFuelLayout(
        grid, request, sourceDirections, fuelLines, mergeableBest,
        mergeableGoal, mergeableGoalReached);
    if (mergeableGoalReached != nullptr && *mergeableGoalReached) {
        return {std::nullopt, FinalizeFailureKind::None};
    }

    if (!mergeableTargetMode && isSearchAccepted(grid, sim)) {
        FinalizeResult finalResult = acceptedResultFromImprovedGrid(
            grid, sim, request, sourceDirections, fuelLines, "finalCompactValidationFailed");
        if (finalResult.result.has_value()) {
            return finalResult;
        }
    }
    if (!mergeableTargetMode) {
        if (std::optional<FinalizeResult> specialResult =
            trySpecialManaDustFinalization(grid, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
            return std::move(*specialResult);
        }
    }
    fillSupportBlocks(grid, &supportOptions, protectedPositions);
    Grid filledBridgeBase = grid;
    pruneInactiveSupport(filledBridgeBase, protectedPositions);
    sim = simulateMixedFuel(filledBridgeBase);
    considerMergeableSingleFuelLayout(
        filledBridgeBase, request, sourceDirections, fuelLines,
        mergeableBest, mergeableGoal, mergeableGoalReached);
    if (mergeableGoalReached != nullptr && *mergeableGoalReached) {
        return {std::nullopt, FinalizeFailureKind::None};
    }
    if (!mergeableTargetMode &&
        isSearchAccepted(filledBridgeBase, sim)) {
        FinalizeResult finalResult = acceptedResultFromImprovedGrid(
            std::move(filledBridgeBase), sim, request, sourceDirections, fuelLines,
            "finalCompactValidationFailed");
        if (finalResult.result.has_value()) {
            return finalResult;
        }
    }
    if (!mergeableTargetMode) {
        if (std::optional<FinalizeResult> highHeatResult =
                tryHighHeatSingleFuelFinalization(
                filledBridgeBase, sim, request, sourceDirections, fuelLines,
                cancelRequested)) {
            return std::move(*highHeatResult);
        }
        if (std::optional<FinalizeResult> specialResult =
            trySpecialManaDustFinalization(filledBridgeBase, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
            return std::move(*specialResult);
        }
    }
    Grid improved = improveSupportBlocks(std::move(grid), cancelRequested, kDefaultImproveOptions, &supportOptions,
                                         protectedPositions, true);
    pruneInactiveSupport(improved, protectedPositions);
    sim = simulateMixedFuel(improved);
    considerMergeableSingleFuelLayout(
        improved, request, sourceDirections, fuelLines, mergeableBest,
        mergeableGoal, mergeableGoalReached);
    if (mergeableGoalReached != nullptr && *mergeableGoalReached) {
        return {std::nullopt, FinalizeFailureKind::None};
    }
    if (!mergeableTargetMode) {
        if (std::optional<FinalizeResult> specialResult =
            trySpecialManaDustFinalization(improved, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
            return std::move(*specialResult);
        }
    }
    if (classifyFinalizationFailure(improved, sim, request) == FinalizeFailureKind::CoolingDeficit) {
        const Grid protectedBaseline = improved;
        const std::vector<SourcePrimingTarget> expectedSourceTargets =
            sourcePrimingTargets(improved);
        CoolingExpansionOptions expansionOptions = kCoolingExpansionOptions;
        if (isSpecialManaDustRequest(request)) {
            expansionOptions.handoffCoolingDeficit =
                kManaDustFallbackCoolingCapacity;
        }
        improved = expandCoolingWithPreserver(
            std::move(improved),
            [protectedPositions, protectedBaseline,
             expectedSourceTargets](Grid& candidate) {
                if (!matchesSourcePrimingTargets(
                        candidate, expectedSourceTargets)) {
                    return false;
                }
                if (protectedPositions == nullptr ||
                    protectedPositions->size() != static_cast<size_t>(candidate.volume())) {
                    return true;
                }
                for (const Pos& pos : candidate.interiorPositions()) {
                    const int idx = candidate.index(pos.x, pos.y, pos.z);
                    if (protectedPositions->at(static_cast<size_t>(idx)) &&
                        (candidate.atIndex(idx).kind != protectedBaseline.atIndex(idx).kind ||
                         candidate.atIndex(idx).type != protectedBaseline.atIndex(idx).type)) {
                        return false;
                    }
                }
                return true;
            },
            cancelRequested, expansionOptions);
        pruneInactiveSupport(improved, protectedPositions);
        sim = simulateMixedFuel(improved);
        considerMergeableSingleFuelLayout(
            improved, request, sourceDirections, fuelLines,
            mergeableBest, mergeableGoal, mergeableGoalReached);
        if (mergeableGoalReached != nullptr && *mergeableGoalReached) {
            return {std::nullopt, FinalizeFailureKind::None};
        }
    }
    if (!mergeableTargetMode) {
        if (std::optional<FinalizeResult> specialResult =
            trySpecialManaDustFinalization(improved, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
            return std::move(*specialResult);
        }
    }
    if (!mergeableTargetMode &&
        isSpecialManaDustRequest(request) &&
        hasSpecialManaDustCoolingDeficit(sim) &&
        classifyFinalizationFailure(improved, sim, request) ==
            FinalizeFailureKind::CoolingDeficit) {
        const Grid protectedBaseline = improved;
        const std::vector<SourcePrimingTarget> expectedSourceTargets =
            sourcePrimingTargets(improved);
        improved = expandCoolingWithPreserver(
            std::move(improved),
            [protectedPositions, protectedBaseline,
             expectedSourceTargets](Grid& candidate) {
                if (!matchesSourcePrimingTargets(
                        candidate, expectedSourceTargets)) {
                    return false;
                }
                if (protectedPositions == nullptr ||
                    protectedPositions->size() != static_cast<size_t>(candidate.volume())) {
                    return true;
                }
                for (const Pos& pos : candidate.interiorPositions()) {
                    const int idx = candidate.index(pos.x, pos.y, pos.z);
                    if (protectedPositions->at(static_cast<size_t>(idx)) &&
                        (candidate.atIndex(idx).kind != protectedBaseline.atIndex(idx).kind ||
                         candidate.atIndex(idx).type != protectedBaseline.atIndex(idx).type)) {
                        return false;
                    }
                }
                return true;
            },
            cancelRequested, kCoolingExpansionOptions);
        pruneInactiveSupport(improved, protectedPositions);
        sim = simulateMixedFuel(improved);
        considerMergeableSingleFuelLayout(
            improved, request, sourceDirections, fuelLines,
            mergeableBest, mergeableGoal, mergeableGoalReached);
    }
    if (std::optional<FinalizeResult> bridgeResult =
            tryConductorBridgeFinalization(
                improved, sim, request, sourceDirections, fuelLines,
                protectedPositions, cancelRequested)) {
        if (!mergeableTargetMode) {
            return std::move(*bridgeResult);
        }
        considerMergeableSingleFuelLayout(
            bridgeResult->result->grid, request, sourceDirections,
            fuelLines, mergeableBest, mergeableGoal,
            mergeableGoalReached);
        if (mergeableGoalReached != nullptr && *mergeableGoalReached) {
            return {std::nullopt, FinalizeFailureKind::None};
        }
    }
    if (mergeableGoalReached != nullptr && *mergeableGoalReached) {
        return {std::nullopt, FinalizeFailureKind::None};
    }
    if (mergeableTargetMode) {
        return {std::nullopt, classifyFinalizationFailure(improved, sim, request)};
    }
    if (!sim.compatible || sim.minClusterMargin < 0 || sim.disconnectedFunctionalBlocks != 0 ||
        !hasSafeFuelFlux(improved, sim)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("improvedNotAccepted", improved, &sim, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, classifyFinalizationFailure(improved, sim, request)};
    }

    return acceptedResultFromImprovedGrid(std::move(improved), sim, request, sourceDirections,
                                          fuelLines, "finalCompactValidationFailed");
}

class OptimizationStrategy {
public:
    virtual ~OptimizationStrategy() = default;
    virtual OptimizationResult optimize(const BuildRequest& request, const std::atomic_bool* cancelRequested) const = 0;
};

OptimizationResult optimizeSingleFuelDirectionalLayout(const BuildRequest& request,
                                                       const std::vector<std::vector<int>>& sourceCombos,
                                                       const std::atomic_bool* cancelRequested) {
    const Fuel& fuel = fuels().at(static_cast<size_t>(request.fuelIndices.front()));
    const std::vector<Dimension> dims = singleFuelSearchDimensions();

    for (const Dimension& dim : dims) {
        for (const std::vector<int>& sourceDirections : sourceCombos) {
            throwIfCancelled(cancelRequested);
            SingleFuelSkeletonSearch search{
                &fuel,
                &request,
                &dim,
                &sourceDirections,
                nullptr,
                cancelRequested,
                nullptr,
                nullptr,
                nullptr,
                1,
            };
            if (std::optional<OptimizationResult> result = searchSingleFuelSkeletonSpecs(search)) {
                return std::move(*result);
            }
        }
    }

    throw std::runtime_error("无满足输入要求的搭建方法。");
}

BuildRequest singleFuelRequestForSlot(const BuildRequest& request, int slot) {
    BuildRequest single;
    single.fuelIndices = {request.fuelIndices.at(static_cast<size_t>(slot))};
    single.selectedModeratorTypeIndices = request.selectedModeratorTypeIndices;
    single.selectedReflectorTypeIndices = request.selectedReflectorTypeIndices;
    return single;
}

OptimizationResult optimizeSingleFuelForSlot(const BuildRequest& request, int slot,
                                             const std::atomic_bool* cancelRequested) {
    BuildRequest single = singleFuelRequestForSlot(request, slot);
    const Fuel& fuel = fuels().at(static_cast<size_t>(single.fuelIndices.front()));
    if (fuel.selfPriming) {
        return optimizeSingleFuelDirectionalLayout(single, {{}}, cancelRequested);
    }

    const std::vector<std::vector<int>> sourceCombos = sourceDirectionCombinations(requiredSourceCountForFuels(single));
    return optimizeSingleFuelDirectionalLayout(single, sourceCombos, cancelRequested);
}

MergeableSingleFuelLayout optimizeMergeableSingleFuelForSlot(
    const BuildRequest& request, int slot,
    const MergeableSingleFuelSearchGoal& goal,
    const std::atomic_bool* cancelRequested) {
    BuildRequest single = singleFuelRequestForSlot(request, slot);
    const Fuel& fuel =
        fuels().at(static_cast<size_t>(single.fuelIndices.front()));
    const std::vector<std::vector<int>> sourceCombos =
        fuel.selfPriming
            ? std::vector<std::vector<int>>{{}}
            : sourceDirectionCombinations(
                  requiredSourceCountForFuels(single));
    std::optional<MergeableSingleFuelLayout> best;
    bool goalReached = false;

    for (const Dimension& dim : singleFuelSearchDimensions()) {
        for (const std::vector<int>& sourceDirections : sourceCombos) {
            throwIfCancelled(cancelRequested);
            SingleFuelSkeletonSearch search{
                &fuel,
                &single,
                &dim,
                &sourceDirections,
                nullptr,
                cancelRequested,
                &best,
                &goal,
                &goalReached,
                1,
            };
            searchSingleFuelSkeletonSpecs(search);
            if (goalReached && best.has_value()) {
                return std::move(*best);
            }
        }
    }

    throw std::runtime_error(
        "单燃料子结构无法达到双燃料阶段冷却目标。");
}

bool heatPriorityLess(int lhsSlot, int rhsSlot, const BuildRequest& request) {
    const Fuel& lhs = fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(lhsSlot))));
    const Fuel& rhs = fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(rhsSlot))));
    if (lhs.heat != rhs.heat) {
        return lhs.heat > rhs.heat;
    }
    if (lhs.criticality != rhs.criticality) {
        return lhs.criticality > rhs.criticality;
    }
    return lhsSlot < rhsSlot;
}

} // namespace ncfr::optimizer_detail
