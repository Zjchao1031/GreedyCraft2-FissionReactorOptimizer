#include "OptimizerDetail.h"

#include "Perf.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ncfr::optimizer_detail {

std::vector<const SinkType*> coolingExpansionSinkTypes() {
    std::vector<const SinkType*> sinks;
    sinks.reserve(sinkTypes().size());
    for (const SinkType& sink : sinkTypes()) {
        if (sink.cooling > 0) {
            sinks.push_back(&sink);
        }
    }
    std::sort(sinks.begin(), sinks.end(), [](const SinkType* lhs, const SinkType* rhs) {
        if (lhs->cooling != rhs->cooling) {
            return lhs->cooling > rhs->cooling;
        }
        return lhs->index < rhs->index;
    });
    return sinks;
}

struct CoolingExpansionCandidate {
    Pos pos;
    const SinkType* sink = nullptr;
    Pos bridgePos{};
    const SinkType* bridgeSink = nullptr;
    bool hasBridge = false;
};

struct CoolingExpansionPositionSet {
    std::vector<Pos> direct;
    std::vector<Pos> bridgeTargets;
};

RuleContext ruleContextFromSimulation(const FuelSimulation& sim) {
    RuleContext context;
    context.validSinks = &sim.validSinks;
    context.functionalCells = &sim.functionalCells;
    context.activeModerators = &sim.activeModerators;
    context.activeReflectors = &sim.activeReflectors;
    context.functionalShields = &sim.functionalShields;
    context.functionalIrradiators = &sim.functionalIrradiators;
    return context;
}

bool isCoolingExpansionAnchor(const Grid& grid, const FuelSimulation& sim, int idx) {
    if (!sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
        return false;
    }
    const Block& block = grid.atIndex(idx);
    switch (block.kind) {
    case BlockKind::FuelCell:
        return sim.functionalCells.at(static_cast<size_t>(idx));
    case BlockKind::Moderator:
        return sim.activeModerators.at(static_cast<size_t>(idx));
    case BlockKind::Reflector:
        return sim.activeReflectors.at(static_cast<size_t>(idx));
    case BlockKind::Irradiator:
        return sim.functionalIrradiators.at(static_cast<size_t>(idx));
    case BlockKind::Conductor:
        return true;
    case BlockKind::Sink:
        return sim.validSinks.at(static_cast<size_t>(idx));
    case BlockKind::Shield:
        return sim.functionalShields.at(static_cast<size_t>(idx));
    default:
        return false;
    }
}

bool samePos(const Pos& lhs, const Pos& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool connectsToHeatingCluster(const Grid& grid, const FuelSimulation& sim, const Pos& pos) {
    bool connected = false;
    grid.forEachNeighbor6(pos, [&](const Pos& n) {
        if (!grid.inBounds(n.x, n.y, n.z)) {
            return;
        }
        const int idx = grid.index(n.x, n.y, n.z);
        if (sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            connected = true;
        }
    });
    return connected;
}

CoolingExpansionPositionSet coolingExpansionPositions(const Grid& grid, const FuelSimulation& sim,
                                                      const CoolingExpansionOptions& options) {
    StateVector directMarked(static_cast<size_t>(grid.volume()), false);
    StateVector bridgeMarked(static_cast<size_t>(grid.volume()), false);
    CoolingExpansionPositionSet positions;
    for (int idx = 0; idx < grid.volume(); ++idx) {
        if (!isCoolingExpansionAnchor(grid, sim, idx)) {
            continue;
        }
        const int x = idx % grid.width();
        const int yz = idx / grid.width();
        const int y = yz % grid.height();
        const int z = yz / grid.height();
        grid.forEachNeighbor6({x, y, z}, [&](const Pos& n) {
            if (!grid.isInterior(n.x, n.y, n.z) || !connectsToHeatingCluster(grid, sim, n)) {
                return;
            }
            const int nIdx = grid.index(n.x, n.y, n.z);
            if (directMarked.at(static_cast<size_t>(nIdx))) {
                return;
            }
            const Block& block = grid.atIndex(nIdx);
            if (block.kind == BlockKind::Empty && positions.direct.size() < options.positionLimit) {
                directMarked.at(static_cast<size_t>(nIdx)) = true;
                positions.direct.push_back(n);
            }
        });
        if (positions.direct.size() >= options.positionLimit) {
            break;
        }
    }
    if (options.radius > 1) {
        for (const Pos& bridge : positions.direct) {
            grid.forEachNeighbor6(bridge, [&](const Pos& n) {
                if (!grid.isInterior(n.x, n.y, n.z) || connectsToHeatingCluster(grid, sim, n)) {
                    return;
                }
                const int nIdx = grid.index(n.x, n.y, n.z);
                if (directMarked.at(static_cast<size_t>(nIdx)) ||
                    bridgeMarked.at(static_cast<size_t>(nIdx))) {
                    return;
                }
                const Block& block = grid.atIndex(nIdx);
                if (block.kind == BlockKind::Empty && positions.bridgeTargets.size() < options.bridgeTargetLimit) {
                    bridgeMarked.at(static_cast<size_t>(nIdx)) = true;
                    positions.bridgeTargets.push_back(n);
                }
            });
            if (positions.bridgeTargets.size() >= options.bridgeTargetLimit) {
                break;
            }
        }
    }
    return positions;
}

long long coolingCandidateCooling(const CoolingExpansionCandidate& candidate) {
    long long cooling = candidate.sink != nullptr ? candidate.sink->cooling : 0;
    if (candidate.hasBridge && candidate.bridgeSink != nullptr) {
        cooling += candidate.bridgeSink->cooling;
    }
    return cooling;
}

bool betterCoolingExpansionCandidate(const CoolingExpansionCandidate& lhs,
                                     const CoolingExpansionCandidate& rhs) {
    const long long lhsCooling = coolingCandidateCooling(lhs);
    const long long rhsCooling = coolingCandidateCooling(rhs);
    if (lhsCooling != rhsCooling) {
        return lhsCooling > rhsCooling;
    }
    if (lhs.hasBridge != rhs.hasBridge) {
        return !lhs.hasBridge;
    }
    if (lhs.pos.z != rhs.pos.z) return lhs.pos.z < rhs.pos.z;
    if (lhs.pos.y != rhs.pos.y) return lhs.pos.y < rhs.pos.y;
    if (lhs.pos.x != rhs.pos.x) return lhs.pos.x < rhs.pos.x;
    if (lhs.sink != rhs.sink) return lhs.sink->index < rhs.sink->index;
    if (lhs.bridgePos.z != rhs.bridgePos.z) return lhs.bridgePos.z < rhs.bridgePos.z;
    if (lhs.bridgePos.y != rhs.bridgePos.y) return lhs.bridgePos.y < rhs.bridgePos.y;
    if (lhs.bridgePos.x != rhs.bridgePos.x) return lhs.bridgePos.x < rhs.bridgePos.x;
    if (lhs.bridgeSink == nullptr || rhs.bridgeSink == nullptr) {
        return lhs.bridgeSink == nullptr && rhs.bridgeSink != nullptr;
    }
    return lhs.bridgeSink->index < rhs.bridgeSink->index;
}

void trimCoolingExpansionCandidates(std::vector<CoolingExpansionCandidate>& candidates, size_t limit) {
    if (limit == 0 || candidates.size() <= limit) {
        return;
    }
    std::sort(candidates.begin(), candidates.end(), betterCoolingExpansionCandidate);
    candidates.resize(limit);
}

std::vector<CoolingExpansionCandidate> coolingExpansionCandidates(
    const Grid& grid, const FuelSimulation& sim, const std::function<bool(Grid&)>& preserveGrid,
    const CoolingExpansionPositionSet& positions, const std::vector<const SinkType*>& sinks,
    const CoolingExpansionOptions& options,
#ifndef NDEBUG
    CoolingExpansionPassStats& stats,
#endif
    const std::atomic_bool* cancelRequested) {
    std::vector<CoolingExpansionCandidate> singleCandidates;
    std::vector<CoolingExpansionCandidate> bridgeCandidates;
    RuleContext context = ruleContextFromSimulation(sim);
    for (const Pos& pos : positions.direct) {
        throwIfCancelled(cancelRequested);
        std::vector<const SinkType*> validAtPosition;
        for (const SinkType* sink : sinks) {
            throwIfCancelled(cancelRequested);
#ifndef NDEBUG
            ++stats.ruleChecks;
#endif
            Grid trial = grid;
            trial.at(pos.x, pos.y, pos.z) = {BlockKind::Sink, sink->index};
            if (!preserveGrid(trial)) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(stats, "restoreLineFailed");
#endif
                continue;
            }
            if (!isSinkValidAt(trial, pos, context)) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(stats, "invalidNewSink");
#endif
                continue;
            }
#ifndef NDEBUG
            ++stats.ruleValidSinks;
#endif
            validAtPosition.push_back(sink);
        }

        if (validAtPosition.size() > options.sinkTypeLimit) {
            validAtPosition.resize(options.sinkTypeLimit);
        }
        for (const SinkType* sink : validAtPosition) {
            singleCandidates.push_back({pos, sink});
        }
    }
#ifndef NDEBUG
    stats.singleCandidates = static_cast<long long>(singleCandidates.size());
#endif

    struct BridgeSlot {
        Pos targetPos;
        Pos bridgePos;
    };
    std::vector<BridgeSlot> bridgeSlots;
    bridgeSlots.reserve(options.bridgeTargetCandidateLimit);
    for (const Pos& targetPos : positions.bridgeTargets) {
        throwIfCancelled(cancelRequested);
        grid.forEachNeighbor6(targetPos, [&](const Pos& bridgePos) {
            if (bridgeSlots.size() >= options.bridgeTargetCandidateLimit ||
                !grid.isInterior(bridgePos.x, bridgePos.y, bridgePos.z) ||
                !connectsToHeatingCluster(grid, sim, bridgePos)) {
                return;
            }
            const Block& block = grid.at(bridgePos.x, bridgePos.y, bridgePos.z);
            if (block.kind == BlockKind::Empty && !samePos(bridgePos, targetPos)) {
                bridgeSlots.push_back({targetPos, bridgePos});
            }
        });
        if (bridgeSlots.size() >= options.bridgeTargetCandidateLimit) {
            break;
        }
    }
#ifndef NDEBUG
    stats.bridgeTargetCandidates = static_cast<long long>(bridgeSlots.size());
#endif

    for (const BridgeSlot& slot : bridgeSlots) {
        throwIfCancelled(cancelRequested);
        Grid slotBase = grid;
        slotBase.at(slot.targetPos.x, slot.targetPos.y, slot.targetPos.z) = {BlockKind::Sink, 0};
        slotBase.at(slot.bridgePos.x, slot.bridgePos.y, slot.bridgePos.z) = {BlockKind::Sink, 0};
        if (!preserveGrid(slotBase)) {
#ifndef NDEBUG
            recordCoolingExpansionRejection(stats, "restoreLineFailed");
#endif
            continue;
        }
        if (slotBase.at(slot.targetPos.x, slot.targetPos.y, slot.targetPos.z).kind != BlockKind::Sink ||
            slotBase.at(slot.bridgePos.x, slot.bridgePos.y, slot.bridgePos.z).kind != BlockKind::Sink) {
#ifndef NDEBUG
            recordCoolingExpansionRejection(stats, "restoreLineFailed");
#endif
            continue;
        }

        StateVector optimisticValidSinks = sim.validSinks;
        optimisticValidSinks.at(static_cast<size_t>(
            slotBase.index(slot.targetPos.x, slot.targetPos.y, slot.targetPos.z))) = true;
        optimisticValidSinks.at(static_cast<size_t>(
            slotBase.index(slot.bridgePos.x, slot.bridgePos.y, slot.bridgePos.z))) = true;
        RuleContext pairContext = context;
        pairContext.validSinks = &optimisticValidSinks;

        std::vector<CoolingExpansionCandidate> slotCandidates;
        for (const SinkType* targetSink : sinks) {
            for (const SinkType* bridgeSink : sinks) {
                throwIfCancelled(cancelRequested);
#ifndef NDEBUG
                ++stats.bridgeRuleChecks;
#endif
                Grid trial = slotBase;
                trial.at(slot.targetPos.x, slot.targetPos.y, slot.targetPos.z) =
                    {BlockKind::Sink, targetSink->index};
                trial.at(slot.bridgePos.x, slot.bridgePos.y, slot.bridgePos.z) =
                    {BlockKind::Sink, bridgeSink->index};
                if (!isSinkValidAt(trial, slot.targetPos, pairContext)) {
#ifndef NDEBUG
                    recordCoolingExpansionRejection(stats, "invalidBridgeTarget");
#endif
                    continue;
                }
                if (!isSinkValidAt(trial, slot.bridgePos, pairContext)) {
#ifndef NDEBUG
                    recordCoolingExpansionRejection(stats, "invalidBridgeSink");
#endif
                    continue;
                }
#ifndef NDEBUG
                ++stats.bridgeRuleValidSinks;
#endif
                slotCandidates.push_back({slot.targetPos, targetSink, slot.bridgePos, bridgeSink, true});
            }
        }
        trimCoolingExpansionCandidates(slotCandidates, options.bridgeSinkTypeLimit);
        bridgeCandidates.insert(bridgeCandidates.end(), slotCandidates.begin(), slotCandidates.end());
        trimCoolingExpansionCandidates(bridgeCandidates, options.bridgeCandidateLimit);
    }
#ifndef NDEBUG
    stats.bridgeCandidates = static_cast<long long>(bridgeCandidates.size());
#endif

    std::vector<CoolingExpansionCandidate> candidates;
    candidates.reserve(singleCandidates.size() + bridgeCandidates.size());
    candidates.insert(candidates.end(), singleCandidates.begin(), singleCandidates.end());
    candidates.insert(candidates.end(), bridgeCandidates.begin(), bridgeCandidates.end());
    std::sort(candidates.begin(), candidates.end(), [](const CoolingExpansionCandidate& lhs,
                                                       const CoolingExpansionCandidate& rhs) {
        return betterCoolingExpansionCandidate(lhs, rhs);
    });
#ifndef NDEBUG
    stats.selectedCandidates = static_cast<long long>(candidates.size());
#endif
    return candidates;
}

bool preservesDirectionalCandidate(const Grid& grid, const FuelSimulation& sim, const BuildRequest& request) {
    (void)request;
    return isPreCompactRunnable(sim) && hasSafeFuelFlux(grid, sim) && sim.disconnectedFunctionalBlocks == 0;
}

bool coolingExpansionStartAllowed(const Grid& grid, const FuelSimulation& sim,
                                  bool allowDisconnectedFunctionalBlocks) {
    return isPreCompactRunnable(sim) && hasSafeFuelFlux(grid, sim) &&
           (allowDisconnectedFunctionalBlocks || sim.disconnectedFunctionalBlocks == 0);
}

bool coolingExpansionGoalReached(const FuelSimulation& sim, bool allowDisconnectedFunctionalBlocks) {
    return sim.compatible && sim.minClusterMargin >= 0 &&
           (allowDisconnectedFunctionalBlocks || sim.disconnectedFunctionalBlocks == 0);
}

bool coolingExpansionMakesProgress(const FuelSimulation& trialSim, const FuelSimulation& currentSim,
                                   bool allowDisconnectedFunctionalBlocks) {
    if (!currentSim.compatible && trialSim.compatible) {
        return true;
    }
    if (allowDisconnectedFunctionalBlocks &&
        trialSim.disconnectedFunctionalBlocks < currentSim.disconnectedFunctionalBlocks &&
        trialSim.minClusterMargin >= currentSim.minClusterMargin) {
        return true;
    }
    return trialSim.minClusterMargin > currentSim.minClusterMargin;
}

bool betterCoolingExpansionSimulation(const FuelSimulation& lhs, const FuelSimulation& rhs,
                                      bool allowDisconnectedFunctionalBlocks) {
    if (lhs.compatible != rhs.compatible) {
        return lhs.compatible;
    }
    if (lhs.minClusterMargin != rhs.minClusterMargin) {
        return lhs.minClusterMargin > rhs.minClusterMargin;
    }
    if (allowDisconnectedFunctionalBlocks &&
        lhs.disconnectedFunctionalBlocks != rhs.disconnectedFunctionalBlocks) {
        return lhs.disconnectedFunctionalBlocks < rhs.disconnectedFunctionalBlocks;
    }
    return lhs.cooling > rhs.cooling;
}

Grid expandCoolingWithPreserver(Grid grid, const std::function<bool(Grid&)>& preserveGrid,
                                const std::atomic_bool* cancelRequested,
                                const CoolingExpansionOptions& options,
                                bool allowDisconnectedFunctionalBlocks) {
    FuelSimulation currentSim = simulateMixedFuel(grid);
    if (!coolingExpansionStartAllowed(grid, currentSim, allowDisconnectedFunctionalBlocks) ||
        coolingExpansionGoalReached(currentSim, allowDisconnectedFunctionalBlocks)) {
        return grid;
    }
#ifndef NDEBUG
    logCoolingExpansionCheckpoint("start", grid, currentSim);
#endif

    const std::vector<const SinkType*> sinks = coolingExpansionSinkTypes();
    for (int pass = 0; pass < options.maxPasses; ++pass) {
        throwIfCancelled(cancelRequested);
        const CoolingExpansionPositionSet positions = coolingExpansionPositions(grid, currentSim, options);
        if ((positions.direct.empty() && positions.bridgeTargets.empty()) || sinks.empty()) {
#ifndef NDEBUG
            logCoolingExpansionCheckpoint("noCandidates", grid, currentSim, pass);
#endif
            break;
        }
#ifndef NDEBUG
        CoolingExpansionPassStats passStats;
        passStats.directPositions = positions.direct.size();
        passStats.bridgeTargetPositions = positions.bridgeTargets.size();
        passStats.positions = passStats.directPositions + passStats.bridgeTargetPositions;
        passStats.clusterConnectedPositions = static_cast<size_t>(std::count_if(
            positions.direct.begin(), positions.direct.end(), [&](const Pos& pos) {
                return connectsToHeatingCluster(grid, currentSim, pos);
            }));
        passStats.sinkTypes = sinks.size();
#endif
        const std::vector<CoolingExpansionCandidate> candidates = coolingExpansionCandidates(
            grid, currentSim, preserveGrid, positions, sinks,
            options,
#ifndef NDEBUG
            passStats,
#endif
            cancelRequested);
        if (candidates.empty()) {
#ifndef NDEBUG
            logCoolingExpansionStats("noRuleValidCandidates", grid, currentSim, pass, passStats);
            logCoolingExpansionCheckpoint("noCandidates", grid, currentSim, pass);
#endif
            break;
        }

        bool found = false;
        Grid bestGrid = grid;
        FuelSimulation bestSim = currentSim;
        Pos bestPos{};
        Pos bestBridgePos{};
        [[maybe_unused]] const SinkType* bestSink = nullptr;
        [[maybe_unused]] const SinkType* bestBridgeSink = nullptr;
        [[maybe_unused]] bool bestHasBridge = false;
        for (const CoolingExpansionCandidate& candidate : candidates) {
            throwIfCancelled(cancelRequested);
#ifndef NDEBUG
            ++passStats.trials;
            if (candidate.hasBridge) {
                ++passStats.bridgeTrials;
            }
#endif
            const Pos& pos = candidate.pos;
            const SinkType* sink = candidate.sink;
            Grid trial = grid;
            trial.at(pos.x, pos.y, pos.z) = {BlockKind::Sink, sink->index};
            if (candidate.hasBridge) {
                trial.at(candidate.bridgePos.x, candidate.bridgePos.y, candidate.bridgePos.z) =
                    {BlockKind::Sink, candidate.bridgeSink->index};
            }
            if (!preserveGrid(trial)) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(passStats, "restoreLineFailed");
#endif
                continue;
            }
            FuelSimulation trialSim = simulateMixedFuel(trial);
            if (!trialSim.validSinks.at(static_cast<size_t>(trial.index(pos.x, pos.y, pos.z)))) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(passStats,
                                                candidate.hasBridge ? "invalidBridgeTarget" : "invalidNewSink",
                                                &trialSim);
#endif
                continue;
            }
            if (candidate.hasBridge &&
                !trialSim.validSinks.at(static_cast<size_t>(trial.index(candidate.bridgePos.x,
                                                                        candidate.bridgePos.y,
                                                                        candidate.bridgePos.z)))) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(passStats, "invalidBridgeSink", &trialSim);
#endif
                continue;
            }
            if (!isPreCompactRunnable(trialSim)) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(passStats, "notRunnable", &trialSim);
#endif
                continue;
            }
            if (!hasSafeFuelFlux(trial, trialSim)) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(passStats, "unsafeFlux", &trialSim);
#endif
                continue;
            }
            if (!allowDisconnectedFunctionalBlocks && trialSim.disconnectedFunctionalBlocks != 0) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(passStats, "disconnected", &trialSim);
#endif
                continue;
            }
            if (!coolingExpansionMakesProgress(trialSim, currentSim, allowDisconnectedFunctionalBlocks)) {
#ifndef NDEBUG
                recordCoolingExpansionRejection(passStats, "noMarginGain", &trialSim);
#endif
                continue;
            }
            if (betterCoolingExpansionSimulation(trialSim, bestSim, allowDisconnectedFunctionalBlocks)) {
                bestSim = std::move(trialSim);
                bestGrid = std::move(trial);
                bestPos = pos;
                bestSink = sink;
                bestBridgePos = candidate.bridgePos;
                bestBridgeSink = candidate.bridgeSink;
                bestHasBridge = candidate.hasBridge;
                found = true;
#ifndef NDEBUG
                ++passStats.newBest;
                if (candidate.hasBridge) {
                    ++passStats.bridgeNewBest;
                }
#endif
#ifndef NDEBUG
            } else {
                recordCoolingExpansionRejection(passStats, "notBest", &trialSim);
#endif
            }
        }
        if (!found) {
#ifndef NDEBUG
            logCoolingExpansionStats("noImprovement", grid, currentSim, pass, passStats);
            logCoolingExpansionCheckpoint("noImprovement", grid, currentSim, pass);
#endif
            break;
        }
#ifndef NDEBUG
        logCoolingExpansionStats("accept", grid, currentSim, pass, passStats);
        logCoolingExpansionCheckpoint("accept", bestGrid, bestSim, pass, &bestPos, bestSink,
                                      currentSim.minClusterMargin, bestHasBridge ? &bestBridgePos : nullptr,
                                      bestHasBridge ? bestBridgeSink : nullptr);
#endif
        grid = std::move(bestGrid);
        currentSim = std::move(bestSim);
        if (coolingExpansionGoalReached(currentSim, allowDisconnectedFunctionalBlocks)) {
#ifndef NDEBUG
            logCoolingExpansionCheckpoint("success", grid, currentSim, pass);
#endif
            break;
        }
    }
    return grid;
}

Grid expandCooling(Grid grid, const BuildRequest& request, const std::vector<int>& sourceDirections,
                   const std::vector<int>& reflectorDirections,
                   const std::atomic_bool* cancelRequested, const CoolingExpansionOptions& options) {
    return expandCoolingWithPreserver(
        std::move(grid),
        [&](Grid& candidate) {
            return restoreDirectionalFuelLines(candidate, request, sourceDirections, reflectorDirections);
        },
        cancelRequested, options);
}

} // namespace ncfr::optimizer_detail
