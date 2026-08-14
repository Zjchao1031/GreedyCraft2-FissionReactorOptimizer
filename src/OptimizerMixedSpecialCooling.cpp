#include "OptimizerSpecialCooling.h"

#include "OptimizerCommon.h"
#include "OptimizerConductorBridge.h"
#include "OptimizerDiagnostics.h"
#include "OptimizerDirectional.h"
#include "OptimizerSingle.h"
#include "FuelSpecialCases.h"
#include "Perf.h"

#include <algorithm>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
namespace ncfr::optimizer_detail {

std::optional<Grid> tryMixedFuelSpecialCoolingFallback(
    Grid grid, const BuildRequest& request,
    const std::vector<FuelLayoutContext>& fuelContexts,
    const std::atomic_bool* cancelRequested,
    bool allowDisconnectedFunctionalBlocks) {
    FuelSimulation currentSim = simulateMixedFuel(grid);
    const long long initialDeficit =
        currentSim.rawHeating - currentSim.cooling;
    std::vector<bool> coveredSlots(request.fuelIndices.size(), false);
    bool hasCompleteContexts =
        fuelContexts.size() == request.fuelIndices.size();
    for (const FuelLayoutContext& context : fuelContexts) {
        if (context.requestSlot < 0 ||
            context.requestSlot >=
                static_cast<int>(coveredSlots.size()) ||
            coveredSlots.at(
                static_cast<size_t>(context.requestSlot))) {
            hasCompleteContexts = false;
            break;
        }
        coveredSlots.at(static_cast<size_t>(context.requestSlot)) = true;
    }
    if (!hasCompleteContexts ||
        std::find(coveredSlots.begin(), coveredSlots.end(), false) !=
            coveredSlots.end()) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "rejected", grid, currentSim, initialDeficit,
            "incompleteFuelContexts");
#endif
        return std::nullopt;
    }
    if (!isPreCompactRunnable(currentSim)) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "rejected", grid, currentSim, initialDeficit, "notRunnable");
#endif
        return std::nullopt;
    }
    if (!hasSafeFuelFlux(grid, currentSim)) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "rejected", grid, currentSim, initialDeficit, "unsafeFlux");
#endif
        return std::nullopt;
    }
    if (!allowDisconnectedFunctionalBlocks &&
        currentSim.disconnectedFunctionalBlocks != 0) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "rejected", grid, currentSim, initialDeficit, "disconnected");
#endif
        return std::nullopt;
    }
    if (hasInvalidSinks(grid, currentSim)) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "rejected", grid, currentSim, initialDeficit, "invalidSinks");
#endif
        return std::nullopt;
    }

    if (initialDeficit <= 0) {
#ifndef NDEBUG
        logDualFuelCoolingCheckpoint(
            "baseline", grid, currentSim, initialDeficit, false, false,
            "alreadyBalanced");
#endif
        const bool accepted =
            isSearchAccepted(grid, currentSim) ||
            (allowDisconnectedFunctionalBlocks &&
             currentSim.compatible &&
             currentSim.minClusterMargin >= 0 &&
             !hasInvalidSinks(grid, currentSim));
        return accepted
                   ? std::optional<Grid>(std::move(grid))
                   : std::nullopt;
    }
    const MixedFuelSpecialCoolingLimits coolingLimits =
        mixedFuelSpecialCoolingLimits(fuelContexts.size());
    if (initialDeficit > coolingLimits.manaDustDeficitLimit) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "rejected", grid, currentSim, initialDeficit,
            "deficitExceedsManaDustLimit");
#endif
        return std::nullopt;
    }
    const bool allowCarobbiite =
        initialDeficit > coolingLimits.endStoneDeficitLimit;
    const bool allowManaDust =
        initialDeficit > coolingLimits.carobbiiteDeficitLimit;

#ifndef NDEBUG
    logDualFuelCoolingCheckpoint(
        "baseline", grid, currentSim, initialDeficit,
        allowCarobbiite, allowManaDust,
        "fuelContexts=" + std::to_string(fuelContexts.size()));
#endif

    const std::vector<SourcePrimingTarget> expectedSourceTargets =
        sourcePrimingTargets(grid);
    StateVector protectedPositions(
        static_cast<size_t>(grid.volume()), false);
    markOccupiedInteriorProtected(protectedPositions, grid);

    auto connectPlacedSpecialSinks =
        [&](Grid& finalGrid, FuelSimulation& finalSim,
            StateVector& finalProtected,
            const SimulationOptions& connectionOptions,
            const char* checkpoint) {
#ifdef NDEBUG
            (void)checkpoint;
#endif
            struct SpecialSinkConnection {
                Pos pos;
                const char* stage = "";
                int priority = 0;
            };

            const int endStoneType = endStoneSinkType();
            const int carobbiiteType = carobbiiteSinkType();
            std::vector<SpecialSinkConnection> sinks;
            for (const Pos& pos : finalGrid.interiorPositions()) {
                const Block& block =
                    finalGrid.at(pos.x, pos.y, pos.z);
                if (block.kind != BlockKind::Sink) {
                    continue;
                }
                if (block.type == endStoneType) {
                    sinks.push_back({pos, "end_stone", 0});
                } else if (block.type == carobbiiteType) {
                    sinks.push_back({pos, "carobbiite", 1});
                } else if (isManaDustSink(block)) {
                    sinks.push_back({pos, "mana_dust", 2});
                }
            }
            std::sort(
                sinks.begin(), sinks.end(),
                [](const SpecialSinkConnection& lhs,
                   const SpecialSinkConnection& rhs) {
                    if (lhs.priority != rhs.priority) {
                        return lhs.priority < rhs.priority;
                    }
                    if (lhs.pos.z != rhs.pos.z) {
                        return lhs.pos.z < rhs.pos.z;
                    }
                    if (lhs.pos.y != rhs.pos.y) {
                        return lhs.pos.y < rhs.pos.y;
                    }
                    return lhs.pos.x < rhs.pos.x;
                });
            if (sinks.empty()) {
#ifndef NDEBUG
                logDualFuelFallbackCheckpoint(
                    "connectionsRejected", finalGrid, finalSim,
                    initialDeficit, "noSpecialSinks");
#endif
                return false;
            }

            const std::vector<SourcePrimingTarget> sourceTargets =
                sourcePrimingTargets(finalGrid);
            finalSim =
                simulateMixedFuel(finalGrid, connectionOptions);
            for (const SpecialSinkConnection& sink : sinks) {
#ifndef NDEBUG
                HighHeatPlacementFailureStats connectionStats;
#endif
                if (!tryConnectSpecialSinkToHeatingCluster(
                        finalGrid, finalSim, sink.pos, finalProtected,
                        cancelRequested, connectionOptions
#ifndef NDEBUG
                        , &connectionStats
#endif
                        )) {
#ifndef NDEBUG
                    logDualFuelSinkCheckpoint(
                        sink.stage, -1, sink.pos, -1,
                        "connectionFailed", finalGrid, {},
                        &connectionStats);
#endif
                    return false;
                }
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    sink.stage, -1, sink.pos, -1, "connected",
                    finalGrid,
                    "cooling=" + std::to_string(finalSim.cooling));
#endif
            }
            if (!matchesSourcePrimingTargets(finalGrid, sourceTargets)) {
#ifndef NDEBUG
                logDualFuelFallbackCheckpoint(
                    "connectionsRejected", finalGrid, finalSim,
                    initialDeficit, "sourceTargetChanged");
#endif
                return false;
            }

            finalSim = simulateMixedFuel(finalGrid);
#ifndef NDEBUG
            logDualFuelCoolingCheckpoint(
                checkpoint, finalGrid, finalSim, initialDeficit,
                allowCarobbiite, allowManaDust,
                "specialSinks=" + std::to_string(sinks.size()));
#endif
            const bool accepted =
                isSearchAccepted(finalGrid, finalSim) ||
                (allowDisconnectedFunctionalBlocks &&
                 finalSim.compatible &&
                 finalSim.minClusterMargin >= 0 &&
                 !hasInvalidSinks(finalGrid, finalSim));
            if (!accepted) {
#ifndef NDEBUG
                logDualFuelFallbackCheckpoint(
                    "connectionsRejected", finalGrid, finalSim,
                    initialDeficit, "finalValidationFailed");
#endif
                return false;
            }
            return true;
        };

    struct ContextReflectorCandidates {
        const FuelLayoutContext* context = nullptr;
        std::vector<EndStoneReflectorCandidate> endStone;
    };

    std::vector<const FuelLayoutContext*> orderedContexts;
    orderedContexts.reserve(fuelContexts.size());
    for (const FuelLayoutContext& context : fuelContexts) {
        orderedContexts.push_back(&context);
    }
    std::sort(
        orderedContexts.begin(), orderedContexts.end(),
        [&](const FuelLayoutContext* lhs,
            const FuelLayoutContext* rhs) {
            return heatPriorityLess(
                lhs->requestSlot, rhs->requestSlot, request);
        });

    std::vector<ContextReflectorCandidates> reflectorCandidates;
    reflectorCandidates.reserve(orderedContexts.size());
    for (const FuelLayoutContext* context : orderedContexts) {
        throwIfCancelled(cancelRequested);
        if (context->requestSlot < 0 ||
            context->requestSlot >=
                static_cast<int>(request.fuelIndices.size())) {
#ifndef NDEBUG
            logDualFuelFallbackCheckpoint(
                "endStoneContextSkipped", grid, currentSim, initialDeficit,
                "invalidSlot=" + std::to_string(context->requestSlot));
#endif
            continue;
        }

        const auto endStoneCandidates =
            endStoneReflectorSinkCandidates(
                grid, context->fuelPos, context->fuelLines);
        if (!endStoneCandidates.has_value()) {
#ifndef NDEBUG
            logDualFuelSinkCheckpoint(
                "end_stone", context->requestSlot, context->fuelPos, -1,
                "noCandidates", grid, "reflectorLayoutUnavailable");
#endif
            continue;
        }
        reflectorCandidates.push_back(
            {context, *endStoneCandidates});
        for (const EndStoneReflectorCandidate& candidate :
             *endStoneCandidates) {
            throwIfCancelled(cancelRequested);
            const int sinkIdx = grid.index(
                candidate.pos.x, candidate.pos.y,
                candidate.pos.z);
            if (grid.atIndex(sinkIdx).kind != BlockKind::Empty) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "end_stone", context->requestSlot, candidate.pos,
                    candidate.faceDirection, "occupied", grid,
                    "block=" +
                        std::string(blockKindLabel(
                            grid.atIndex(sinkIdx).kind)));
#endif
                continue;
            }

            Grid trial = grid;
            StateVector trialProtected = protectedPositions;
            trial.atIndex(sinkIdx) = {
                BlockKind::Sink, endStoneSinkType()};
            markProtected(
                trialProtected, trial, candidate.pos);
            FuelSimulation trialSim = simulateMixedFuel(trial);
            if (!isPreCompactRunnable(trialSim)) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "end_stone", context->requestSlot, candidate.pos,
                    candidate.faceDirection, "notRunnable", trial);
#endif
                continue;
            }
            if (!hasSafeFuelFlux(trial, trialSim)) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "end_stone", context->requestSlot, candidate.pos,
                    candidate.faceDirection, "unsafeFlux", trial);
#endif
                continue;
            }
            if (!trialSim.validSinks.at(static_cast<size_t>(sinkIdx))) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "end_stone", context->requestSlot, candidate.pos,
                    candidate.faceDirection, "invalidSink", trial);
#endif
                continue;
            }
            if (!matchesSourcePrimingTargets(trial, expectedSourceTargets)) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "end_stone", context->requestSlot, candidate.pos,
                    candidate.faceDirection, "sourceTargetChangedBeforeConnect",
                    trial);
#endif
                continue;
            }
            if (hasInvalidSinks(trial, trialSim)) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "end_stone", context->requestSlot, candidate.pos,
                    candidate.faceDirection, "invalidSinksAfterPlacement", trial);
#endif
                continue;
            }
            grid = std::move(trial);
            currentSim = std::move(trialSim);
            protectedPositions = std::move(trialProtected);
#ifndef NDEBUG
            logDualFuelSinkCheckpoint(
                "end_stone", context->requestSlot, candidate.pos,
                candidate.faceDirection, "placed", grid,
                "cooling=" + std::to_string(currentSim.cooling));
#endif
        }
    }

    currentSim = simulateMixedFuel(grid);
#ifndef NDEBUG
    logDualFuelCoolingCheckpoint(
        "afterEndStone", grid, currentSim, initialDeficit,
        allowCarobbiite, allowManaDust);
#endif
    if (currentSim.cooling >= currentSim.rawHeating) {
        if (connectPlacedSpecialSinks(
                grid, currentSim, protectedPositions, {},
                "afterEndStoneConnections")) {
#ifndef NDEBUG
            logDualFuelFallbackCheckpoint(
                "acceptedAfterEndStone", grid, currentSim,
                initialDeficit);
#endif
            return grid;
        }
        return std::nullopt;
    }
    if (!allowCarobbiite) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "carobbiiteSkipped", grid, currentSim, initialDeficit,
            "initialDeficitAtMostEndStoneLimit");
#endif
        return std::nullopt;
    }

    for (const ContextReflectorCandidates& contextCandidates :
         reflectorCandidates) {
        throwIfCancelled(cancelRequested);
        if (contextCandidates.context == nullptr) {
            continue;
        }
        const FuelLayoutContext& context =
            *contextCandidates.context;
        const std::vector<CarobbiiteReflectorCandidate>
            carobbiiteCandidates =
                carobbiiteReflectorSinkCandidates(
                    grid, context.fuelLines,
                    contextCandidates.endStone);
        if (carobbiiteCandidates.empty()) {
#ifndef NDEBUG
            logDualFuelSinkCheckpoint(
                "carobbiite", context.requestSlot, context.fuelPos, -1,
                "noCandidates", grid, "reflectorLayoutUnavailable");
#endif
        }
        for (const CarobbiiteReflectorCandidate& candidate :
             carobbiiteCandidates) {
            throwIfCancelled(cancelRequested);
            Grid trial = grid;
            FuelSimulation trialSim = currentSim;
            StateVector trialProtected = protectedPositions;
#ifndef NDEBUG
            HighHeatPlacementFailureStats placementStats;
#endif
            if (!tryPlaceCarobbiiteSink(
                    trial, trialSim, trialProtected, candidate
#ifndef NDEBUG
                    , &placementStats
#endif
                    )) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "carobbiite", context.requestSlot, candidate.pos,
                    candidate.faceDirection, "placementFailed", trial,
                    "endStonePos=" + posLabel(candidate.endStonePos),
                    &placementStats);
#endif
                continue;
            }
            if (!matchesSourcePrimingTargets(trial, expectedSourceTargets)) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "carobbiite", context.requestSlot, candidate.pos,
                    candidate.faceDirection, "sourceTargetChangedBeforeConnect",
                    trial, "endStonePos=" + posLabel(candidate.endStonePos));
#endif
                continue;
            }
            if (hasInvalidSinks(trial, trialSim)) {
#ifndef NDEBUG
                logDualFuelSinkCheckpoint(
                    "carobbiite", context.requestSlot, candidate.pos,
                    candidate.faceDirection, "invalidSinksAfterPlacement", trial,
                    "endStonePos=" + posLabel(candidate.endStonePos));
#endif
                continue;
            }
            grid = std::move(trial);
            currentSim = std::move(trialSim);
            protectedPositions =
                std::move(trialProtected);
#ifndef NDEBUG
            logDualFuelSinkCheckpoint(
                "carobbiite", context.requestSlot, candidate.pos,
                candidate.faceDirection, "placed", grid,
                "endStonePos=" + posLabel(candidate.endStonePos) +
                    " cooling=" + std::to_string(currentSim.cooling));
#endif
        }
    }

    currentSim = simulateMixedFuel(grid);
#ifndef NDEBUG
    logDualFuelCoolingCheckpoint(
        "afterCarobbiite", grid, currentSim, initialDeficit,
        allowCarobbiite, allowManaDust);
#endif
    if (currentSim.cooling >= currentSim.rawHeating) {
        if (connectPlacedSpecialSinks(
                grid, currentSim, protectedPositions, {},
                "afterCarobbiiteConnections")) {
#ifndef NDEBUG
            logDualFuelFallbackCheckpoint(
                "acceptedAfterCarobbiite", grid, currentSim,
                initialDeficit);
#endif
            return grid;
        }
        return std::nullopt;
    }
    if (!allowManaDust) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "manaDustSkipped", grid, currentSim, initialDeficit,
            "initialDeficitAtMostCarobbiiteLimit");
#endif
        return std::nullopt;
    }

#ifndef NDEBUG
    logDualFuelCoolingCheckpoint(
        "manaDustStart", grid, currentSim, initialDeficit,
        allowCarobbiite, allowManaDust);
#endif
    ManaDustPreparationResult preparation =
        prepareManaDustFallbackGrid(
            grid, request, fuelContexts,
            ManaDustCompactionStrategy::PreserveMixedFuelSources);
    if (!preparation.grid.has_value()) {
#ifndef NDEBUG
        const char* checkpoint =
            preparation.failure == ManaDustPreparationFailure::Padding
                ? "manaDustPaddingFailed"
                : preparation.failure ==
                          ManaDustPreparationFailure::Preplacement
                      ? "manaDustPreplacementFailed"
                      : "manaDustCompactionFailed";
        logDualFuelFallbackCheckpoint(
            checkpoint, grid, currentSim, initialDeficit,
            preparation.detail);
#endif
        return std::nullopt;
    }
    Grid manaGrid = std::move(*preparation.grid);

    StateVector manaProtected(
        static_cast<size_t>(manaGrid.volume()), false);
    markOccupiedInteriorProtected(manaProtected, manaGrid);
    StateVector forcedValidSinks(
        static_cast<size_t>(manaGrid.volume()), false);
    for (const Pos& sink : manaDustSinkPositions(manaGrid)) {
        const int idx = manaGrid.index(sink.x, sink.y, sink.z);
        manaProtected.at(static_cast<size_t>(idx)) = true;
        forcedValidSinks.at(static_cast<size_t>(idx)) = true;
#ifndef NDEBUG
        logDualFuelSinkCheckpoint(
            "mana_dust", -1, sink, -1, "placed", manaGrid);
#endif
    }

    const std::vector<SourcePrimingTarget> paddedSourceTargets =
        sourcePrimingTargets(manaGrid);
    const SimulationOptions searchOptions{&forcedValidSinks};
    FuelSimulation manaSim =
        simulateMixedFuel(manaGrid, searchOptions);
    if (!isPreCompactRunnable(manaSim)) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "manaDustInitialRejected", manaGrid, manaSim, initialDeficit,
            "notRunnable");
#endif
        return std::nullopt;
    }
    if (!hasSafeFuelFlux(manaGrid, manaSim)) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "manaDustInitialRejected", manaGrid, manaSim, initialDeficit,
            "unsafeFlux");
#endif
        return std::nullopt;
    }
    if (!matchesSourcePrimingTargets(manaGrid, paddedSourceTargets)) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "manaDustSourceTargetChanged", manaGrid, manaSim,
            initialDeficit);
#endif
        return std::nullopt;
    }

    if (!connectPlacedSpecialSinks(
            manaGrid, manaSim, manaProtected, searchOptions,
            "afterManaDustConnections")) {
        return std::nullopt;
    }

    manaSim = simulateMixedFuel(manaGrid);
    const bool manaSinksFunctional =
        hasEightFunctionalManaDustSinks(manaGrid, manaSim);
#ifndef NDEBUG
    logDualFuelCoolingCheckpoint(
        "afterManaDust", manaGrid, manaSim, initialDeficit,
        allowCarobbiite, allowManaDust,
        "manaDustFunctional=" +
            std::to_string(manaSinksFunctional ? 1 : 0));
#endif
    if (!manaSinksFunctional) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "manaDustFinalRejected", manaGrid, manaSim, initialDeficit,
            "manaDustFunctional=" +
                std::to_string(manaSinksFunctional ? 1 : 0));
#endif
        return std::nullopt;
    }
#ifndef NDEBUG
    logDualFuelFallbackCheckpoint(
        "acceptedAfterManaDust", manaGrid, manaSim, initialDeficit);
#endif
    return manaGrid;
}

} // namespace ncfr::optimizer_detail
