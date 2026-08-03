#include "OptimizerSpecialCooling.h"

#include "OptimizerCommon.h"
#include "OptimizerDiagnostics.h"
#include "OptimizerDirectional.h"
#include "OptimizerSingle.h"
#include "FuelSpecialCases.h"
#include "Perf.h"

#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
namespace ncfr::optimizer_detail {

std::optional<FinalizeResult> trySpecialManaDustFinalization(
    const Grid& grid, const FuelSimulation& sim, const BuildRequest& request,
    const std::vector<int>& sourceDirections, const std::vector<FuelLineSpec>& fuelLines,
    const StateVector* protectedPositions, const std::atomic_bool* cancelRequested) {
    if (!isSpecialManaDustRequest(request) ||
        !isPreCompactRunnable(sim) ||
        !hasSafeFuelFlux(grid, sim) ||
        !hasSpecialManaDustCoolingDeficit(sim)) {
        return std::nullopt;
    }

    (void)protectedPositions;
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != request.fuelIndices.size()) {
        return std::nullopt;
    }
    const Pos fuelPos = fuelPositions.front();
    const std::vector<FuelLayoutContext> fuelContexts{
        {0, fuelPos, sourceDirections, fuelLines},
    };
#ifndef NDEBUG
    HighHeatPlacementFailureStats manaDustPlacementStats;
    HighHeatPlacementFailureStats manaDustConnectionStats;
#endif
    ManaDustPreparationResult preparation =
        prepareManaDustFallbackGrid(
            grid, request, fuelContexts,
            ManaDustCompactionStrategy::DirectionalSingleFuel);
    if (!preparation.grid.has_value()) {
#ifndef NDEBUG
        if (preparation.failure ==
            ManaDustPreparationFailure::Padding) {
            NCFR_PERF_CHECKPOINT(
                "manaDust.padding", preparation.detail.c_str());
        } else if (preparation.failure ==
                   ManaDustPreparationFailure::Preplacement) {
            ++manaDustPlacementStats.noCandidates;
            logHighHeatPlacementFailures(
                "mana_dust_place", grid, manaDustPlacementStats,
                preparation.detail);
        }
#endif
        return std::nullopt;
    }
    Grid specialGrid = std::move(*preparation.grid);

    StateVector specialProtected(
        static_cast<size_t>(specialGrid.volume()), false);
    markOccupiedInteriorProtected(specialProtected, specialGrid);
    const std::vector<SourcePrimingTarget> paddedSourceTargets =
        sourcePrimingTargets(specialGrid);
    StateVector forcedValidSinks(
        static_cast<size_t>(specialGrid.volume()), false);
    for (const Pos& sink : manaDustSinkPositions(specialGrid)) {
        const int idx = specialGrid.index(sink.x, sink.y, sink.z);
        forcedValidSinks.at(static_cast<size_t>(idx)) = true;
#ifndef NDEBUG
        logDualFuelSinkCheckpoint(
            "mana_dust", -1, sink, -1, "placed", specialGrid);
#endif
    }

    const SimulationOptions searchOptions{&forcedValidSinks};
    FuelSimulation currentSim = simulateMixedFuel(specialGrid, searchOptions);
    const std::vector<Pos> specialSinkPositions =
        placedSpecialCoolingSinkPositions(specialGrid);
    if (!isPreCompactRunnable(currentSim) ||
        !hasSafeFuelFlux(specialGrid, currentSim) ||
        !matchesSourcePrimingTargets(
            specialGrid, paddedSourceTargets) ||
        !connectSpecialSinksToHeatingCluster(
            specialGrid, currentSim, specialSinkPositions, specialProtected,
            cancelRequested, searchOptions
#ifndef NDEBUG
            , &manaDustConnectionStats
#endif
            ) ||
        !matchesSourcePrimingTargets(
            specialGrid, paddedSourceTargets) ||
        !isSearchOperatingSimulation(specialGrid, currentSim)) {
#ifndef NDEBUG
        logHighHeatPlacementFailures("mana_dust_connect", specialGrid,
                                     manaDustConnectionStats);
#endif
        return std::nullopt;
    }

#ifndef NDEBUG
    logHighHeatPlacementFailures("mana_dust_place", specialGrid,
                                 manaDustPlacementStats,
                                 "placed=8");
    logHighHeatPlacementFailures("mana_dust_connect", specialGrid,
                                 manaDustConnectionStats,
                                 "placed=8 specialSinks=" +
                                     std::to_string(
                                         specialSinkPositions.size()) +
                                     " forcedValid=8");
    {
        std::ostringstream os;
        os << "grid=" << gridInteriorLabel(specialGrid)
           << " rawHeating=" << currentSim.rawHeating
           << " cooling=" << currentSim.cooling
           << " minMargin=" << currentSim.minClusterMargin
           << " disconnected=" << currentSim.disconnectedFunctionalBlocks;
        NCFR_PERF_CHECKPOINT("simulation.search", os.str().c_str());
    }
#endif

    FuelSimulation finalSim = simulateMixedFuel(specialGrid);
    const bool compactedSafe =
        isSafeOperatingSimulation(specialGrid, finalSim);
    if (!compactedSafe ||
        !hasFunctionalSpecialManaDustCornerSinks(specialGrid, finalSim) ||
        !hasRequiredSources(specialGrid, request)) {
#ifndef NDEBUG
        const WallConnectionResult wall =
            evaluateHeatingClusterWallConnections(specialGrid, finalSim);
        std::ostringstream os;
        os << "grid=" << gridInteriorLabel(specialGrid)
           << " accepted=" << (compactedSafe ? 1 : 0)
           << " manaDustValid="
           << (hasFunctionalSpecialManaDustCornerSinks(
                   specialGrid, finalSim) ? 1 : 0)
           << " heatingClusters=" << wall.heatingClusters
           << " wallDisconnected=" << wall.disconnectedHeatingClusters;
        NCFR_PERF_CHECKPOINT("wallConnection.final", os.str().c_str());
#endif
        return std::nullopt;
    }
    OptimizationResult result =
        resultFromSimulation(std::move(specialGrid), request, finalSim);
    return FinalizeResult{
        std::optional<OptimizationResult>(std::move(result)),
        FinalizeFailureKind::None};
}

std::optional<FinalizeResult> tryHighHeatSingleFuelFinalization(
    const Grid& grid, const FuelSimulation& sim, const BuildRequest& request,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines,
    const std::atomic_bool* cancelRequested) {
    if (!isHighHeatSingleFuelFallbackEligible(request, sim) ||
        !isPreCompactRunnable(sim) || !hasSafeFuelFlux(grid, sim)) {
        return std::nullopt;
    }

    const Fuel& fuel = fuels().at(static_cast<size_t>(request.fuelIndices.front()));
    const bool useCarobbiite =
        usesCarobbiiteReflectorCooling(fuel) ||
        usesSpecialManaDustCornerSinks(fuel);
    Grid specialGrid = grid;
    FuelSimulation currentSim = sim;

    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(specialGrid);
    if (fuelPositions.size() != 1) {
        return std::nullopt;
    }
    const Pos fuelPos = fuelPositions.front();
    StateVector specialProtected(static_cast<size_t>(specialGrid.volume()), false);
    markDirectionalLayoutProtected(specialProtected, specialGrid, fuelPos,
                                   sourceDirections, fuelLines);
    markOccupiedInteriorProtected(specialProtected, specialGrid);

    const auto endStoneCandidates =
        endStoneReflectorSinkCandidates(specialGrid, fuelPos, fuelLines);
    if (!endStoneCandidates.has_value()) {
        return std::nullopt;
    }

    const int sinkType = endStoneSinkType();
#ifndef NDEBUG
    logHighHeatCoolingCheckpoint("baseline", specialGrid, currentSim,
                                 endStoneCandidates->size(), 0, 0, 0,
                                 0, 0, 0, 0);
#endif
    std::vector<EndStoneReflectorCandidate> placedEndStoneCandidates;
    std::vector<int> placedEndStoneFaces;
    size_t occupiedEndStonePositions = 0;
#ifndef NDEBUG
    HighHeatPlacementFailureStats endStonePlacementStats;
#endif
    for (const EndStoneReflectorCandidate& candidate : *endStoneCandidates) {
        throwIfCancelled(cancelRequested);
        const Pos& sinkPos = candidate.pos;
        const int sinkIdx = specialGrid.index(sinkPos.x, sinkPos.y, sinkPos.z);
        if (specialGrid.atIndex(sinkIdx).kind != BlockKind::Empty) {
            ++occupiedEndStonePositions;
#ifndef NDEBUG
            ++endStonePlacementStats.occupied;
            logHighHeatPlacementFailures(
                "end_stone_place", specialGrid, endStonePlacementStats,
                "pos=" + posLabel(sinkPos) +
                    " block=" + blockKindLabel(specialGrid.atIndex(sinkIdx).kind));
#endif
            continue;
        }

        specialGrid.atIndex(sinkIdx) = {BlockKind::Sink, sinkType};
        markProtected(specialProtected, specialGrid, sinkPos);
        placedEndStoneCandidates.push_back(candidate);
    }
#ifndef NDEBUG
    if (placedEndStoneCandidates.empty()) {
        ++endStonePlacementStats.noCandidates;
    }
    logHighHeatPlacementFailures(
        "end_stone_place", specialGrid, endStonePlacementStats,
        "placed=" + std::to_string(placedEndStoneCandidates.size()));
#endif

    currentSim = simulateMixedFuel(specialGrid);
    if (!isPreCompactRunnable(currentSim) ||
        !hasSafeFuelFlux(specialGrid, currentSim)) {
        return std::nullopt;
    }

    std::vector<CarobbiiteReflectorCandidate> carobbiiteCandidates;
    std::vector<int> placedCarobbiiteFaces;
    size_t failedCarobbiitePlacements = 0;
#ifndef NDEBUG
    HighHeatPlacementFailureStats carobbiitePlacementStats;
#endif
    if (useCarobbiite) {
        carobbiiteCandidates = carobbiiteReflectorSinkCandidates(
            specialGrid, fuelLines, placedEndStoneCandidates);
        for (const CarobbiiteReflectorCandidate& candidate :
             carobbiiteCandidates) {
            throwIfCancelled(cancelRequested);
            if (tryPlaceCarobbiiteSink(specialGrid, currentSim,
                                       specialProtected, candidate
#ifndef NDEBUG
                                       , &carobbiitePlacementStats
#endif
                                       )) {
                placedCarobbiiteFaces.push_back(candidate.faceDirection);
            } else {
                ++failedCarobbiitePlacements;
            }
        }
#ifndef NDEBUG
        if (carobbiiteCandidates.empty()) {
            ++carobbiitePlacementStats.noCandidates;
        }
        logHighHeatPlacementFailures(
            "carobbiite_place", specialGrid, carobbiitePlacementStats,
            "candidates=" + std::to_string(carobbiiteCandidates.size()) +
                " placed=" + std::to_string(placedCarobbiiteFaces.size()));
#endif
#ifndef NDEBUG
        logHighHeatCoolingCheckpoint("carobbiite", specialGrid, currentSim,
                                     endStoneCandidates->size(),
                                     placedEndStoneCandidates.size(),
                                     occupiedEndStonePositions, 0,
                                     carobbiiteCandidates.size(),
                                     placedCarobbiiteFaces.size(),
                                     failedCarobbiitePlacements, 0);
#endif
    }

    currentSim = simulateMixedFuel(specialGrid);
    if (currentSim.cooling < currentSim.rawHeating) {
        if (isSpecialManaDustRequest(request) &&
            hasSpecialManaDustCoolingDeficit(currentSim)) {
#ifndef NDEBUG
            logHighHeatCoolingCheckpoint(
                "manaDustHandoff", specialGrid, currentSim,
                endStoneCandidates->size(), 0,
                occupiedEndStonePositions, 0,
                carobbiiteCandidates.size(),
                placedCarobbiiteFaces.size(),
                failedCarobbiitePlacements, 0);
#endif
            if (std::optional<FinalizeResult> manaDustResult =
                    trySpecialManaDustFinalization(
                        specialGrid, currentSim, request, sourceDirections,
                        fuelLines, &specialProtected, cancelRequested)) {
                return std::move(*manaDustResult);
            }
        }
        return std::nullopt;
    }

    std::vector<EndStoneReflectorCandidate> unresolvedEndStoneCandidates;
#ifndef NDEBUG
    HighHeatPlacementFailureStats endStoneConnectionStats;
#endif
    for (const EndStoneReflectorCandidate& candidate :
         placedEndStoneCandidates) {
        if (tryConnectSpecialSinkToHeatingCluster(
                specialGrid, currentSim, candidate.pos, specialProtected,
                cancelRequested, {}
#ifndef NDEBUG
                , &endStoneConnectionStats
#endif
                )) {
            placedEndStoneFaces.push_back(candidate.faceDirection);
        } else {
            unresolvedEndStoneCandidates.push_back(candidate);
        }
    }

    size_t failedEndStoneConnections = 0;
    for (const EndStoneReflectorCandidate& candidate :
         unresolvedEndStoneCandidates) {
        const Pos& failedPos = candidate.pos;
        const int failedIdx =
            specialGrid.index(failedPos.x, failedPos.y, failedPos.z);
        if (currentSim.validSinks.at(static_cast<size_t>(failedIdx)) &&
            currentSim.heatingClusterBlocks.at(static_cast<size_t>(failedIdx))) {
            placedEndStoneFaces.push_back(candidate.faceDirection);
            continue;
        }
        specialGrid.atIndex(failedIdx) = {BlockKind::Empty, -1};
        specialProtected.at(static_cast<size_t>(failedIdx)) = false;
        removeCarobbiiteSinksForEndStone(
            specialGrid, specialProtected, placedCarobbiiteFaces,
            carobbiiteCandidates, failedPos);
        ++failedEndStoneConnections;
    }
    currentSim = simulateMixedFuel(specialGrid);
#ifndef NDEBUG
    logHighHeatPlacementFailures(
        "end_stone_connect", specialGrid, endStoneConnectionStats,
        "placedFaces=" + std::to_string(placedEndStoneFaces.size()) +
            " failedConnections=" + std::to_string(failedEndStoneConnections));
    logHighHeatCoolingCheckpoint("endStone", specialGrid, currentSim,
                                 endStoneCandidates->size(),
                                 placedEndStoneFaces.size(),
                                 occupiedEndStonePositions,
                                 failedEndStoneConnections,
                                 carobbiiteCandidates.size(),
                                 placedCarobbiiteFaces.size(),
                                 failedCarobbiitePlacements, 0);
#endif

    const bool prePruneAccepted = isSearchAccepted(specialGrid, currentSim);
#ifndef NDEBUG
    logHighHeatFinalReview("prePrune", specialGrid, currentSim,
                           prePruneAccepted, true, true,
                           placedEndStoneFaces.size(),
                           placedCarobbiiteFaces.size());
#endif
    if (!prePruneAccepted) {
        return std::nullopt;
    }

    pruneInactiveSupport(specialGrid, &specialProtected);
    currentSim = simulateMixedFuel(specialGrid);
    const bool postPruneAccepted = isSearchAccepted(specialGrid, currentSim);
    const bool postPruneEndStoneFunctional =
        placedEndStoneFaces.empty() ||
        hasFunctionalEndStoneSinks(specialGrid, currentSim, fuelLines,
                                   placedEndStoneFaces);
    const bool postPruneCarobbiiteFunctional =
        placedCarobbiiteFaces.empty() ||
        hasFunctionalSpecialCarobbiiteSinks(specialGrid, currentSim,
                                            fuelLines,
                                            placedCarobbiiteFaces);
#ifndef NDEBUG
    logHighHeatFinalReview("postPrune", specialGrid, currentSim,
                           postPruneAccepted, postPruneEndStoneFunctional,
                           postPruneCarobbiiteFunctional,
                           placedEndStoneFaces.size(),
                           placedCarobbiiteFaces.size());
#endif
    if (!postPruneAccepted || !postPruneEndStoneFunctional ||
        !postPruneCarobbiiteFunctional) {
        return std::nullopt;
    }

    FinalizeResult finalResult = acceptedResultFromImprovedGrid(
        std::move(specialGrid), currentSim, request, sourceDirections,
        fuelLines, "highHeatCoolingCompactValidationFailed", true);
    if (!finalResult.result.has_value()) {
#ifndef NDEBUG
        NCFR_PERF_CHECKPOINT("highHeatFinalReview",
                             "reason=finalCompactNoResult");
#endif
        return std::nullopt;
    }
    const FuelSimulation finalSim =
        simulateMixedFuel(finalResult.result->grid);
    const bool finalAccepted =
        isFinalReactorValid(finalResult.result->grid, request, finalSim);
    const bool finalEndStoneFunctional =
        placedEndStoneFaces.empty() ||
        hasFunctionalEndStoneSinks(finalResult.result->grid, finalSim,
                                   fuelLines, placedEndStoneFaces);
    const bool finalCarobbiiteFunctional =
        placedCarobbiiteFaces.empty() ||
        hasFunctionalSpecialCarobbiiteSinks(finalResult.result->grid,
                                            finalSim, fuelLines,
                                            placedCarobbiiteFaces);
#ifndef NDEBUG
    logHighHeatFinalReview("finalResult", finalResult.result->grid, finalSim,
                           finalAccepted, finalEndStoneFunctional,
                           finalCarobbiiteFunctional,
                           placedEndStoneFaces.size(),
                           placedCarobbiiteFaces.size());
#endif
    if (!finalAccepted || !finalEndStoneFunctional ||
        !finalCarobbiiteFunctional) {
        return std::nullopt;
    }
    return finalResult;
}

} // namespace ncfr::optimizer_detail
