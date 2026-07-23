#include "OptimizerDetail.h"

#include "FuelSpecialCases.h"
#include "Perf.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ncfr::optimizer_detail {

bool sameBlockType(const Block& lhs, const Block& rhs) {
    return lhs.kind == rhs.kind && lhs.type == rhs.type;
}

int heatingClusterCount(const FuelSimulation& sim) {
    return static_cast<int>(std::count_if(sim.clusters.begin(), sim.clusters.end(), [](const ClusterStats& cluster) {
        return cluster.rawHeating > 0;
    }));
}

int fuelZSpan(const Grid& grid) {
    int minZ = std::numeric_limits<int>::max();
    int maxZ = std::numeric_limits<int>::min();
    for (const Pos& pos : grid.interiorPositions()) {
        if (grid.at(pos.x, pos.y, pos.z).kind == BlockKind::FuelCell) {
            minZ = std::min(minZ, pos.z);
            maxZ = std::max(maxZ, pos.z);
        }
    }
    if (minZ == std::numeric_limits<int>::max()) {
        return 0;
    }
    return maxZ - minZ;
}

int gridInteriorVolume(const Grid& grid) {
    return grid.internalA() * grid.internalB() * grid.internalC();
}

struct MergeCandidateScore {
    int fuelZSpan = 0;
    int volume = 0;
    int height = 0;
    long long minCoolingMargin = 0;
};

enum class MergeBuildFailureKind {
    None,
    Empty,
    Size,
    Conflict,
    FuelSlot,
    FuelDuplicate,
    FuelMissing,
    Source,
};

enum class MergePhase {
    Planar,
    AnyAxis,
};

enum class MergeFallbackPolicy {
    Disabled,
    SpecialCooling,
};

struct MergeBuildResult {
    std::optional<Grid> grid;
    std::vector<Pos> fuelPositions;
    MergeBuildFailureKind failure = MergeBuildFailureKind::None;
};

struct MergeRejectionSummary {
#ifndef NDEBUG
    std::vector<int> lhsSlots;
    std::vector<int> rhsSlots;
    std::vector<int> requestSlots;
    size_t lhsSinks = 0;
    size_t rhsSinks = 0;
    long long noHeatingSink = 0;
    long long attempts = 0;
    long long planarAttempts = 0;
    long long anyAxisAttempts = 0;
    long long buildEmpty = 0;
    long long buildSize = 0;
    long long buildConflict = 0;
    long long buildFuelSlot = 0;
    long long buildFuelDuplicate = 0;
    long long buildFuelMissing = 0;
    long long buildSource = 0;
    long long simNotRunnable = 0;
    long long simUnsafeFlux = 0;
    long long simDisconnected = 0;
    long long simCooling = 0;
    long long simClusterCount = 0;
    long long simOther = 0;
    long long accepted = 0;
    long long acceptedPlanar = 0;
    long long acceptedAnyAxis = 0;
    bool hasBestRejected = false;
    const char* bestRejectedReason = "none";
    bool bestRejectedCompatible = false;
    bool bestRejectedSafeFlux = false;
    int bestRejectedFuelCells = 0;
    int bestRejectedRunningCells = 0;
    int bestRejectedDisconnected = 0;
    int bestRejectedClusters = 0;
    int bestRejectedUnsafeFluxCells = 0;
    long long bestRejectedMargin = std::numeric_limits<long long>::min();
    long long bestRejectedRawHeating = 0;
    long long bestRejectedCooling = 0;
    int bestRejectedA = 0;
    int bestRejectedB = 0;
    int bestRejectedC = 0;
#endif
};

struct EvaluatedMergeCandidate {
    Grid grid;
    FuelSimulation sim;
    std::vector<FuelLayoutContext> fuelContexts;
};

MergeCandidateScore mergeCandidateScore(const Grid& grid, const FuelSimulation& sim) {
    return {fuelZSpan(grid), gridInteriorVolume(grid), grid.internalC(), sim.minClusterMargin};
}

bool isBetterMergeCandidate(const MergeCandidateScore& candidate, const MergeCandidateScore& currentBest) {
    if (candidate.fuelZSpan != currentBest.fuelZSpan) {
        return candidate.fuelZSpan < currentBest.fuelZSpan;
    }
    if (candidate.volume != currentBest.volume) {
        return candidate.volume < currentBest.volume;
    }
    if (candidate.height != currentBest.height) {
        return candidate.height < currentBest.height;
    }
    return candidate.minCoolingMargin > currentBest.minCoolingMargin;
}

bool hasValidHeatingSinkForMerge(
    const Grid& grid, const FuelSimulation& sim) {
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

bool isStructurallyMergeable(
    const Grid& grid, const FuelSimulation& sim) {
    return isPreCompactRunnable(sim) &&
           hasSafeFuelFlux(grid, sim) &&
           sim.disconnectedFunctionalBlocks == 0 &&
           !hasInvalidSinks(grid, sim) &&
           heatingClusterCount(sim) == 1 &&
           hasValidHeatingSinkForMerge(grid, sim);
}

bool betterCoolingDeficitMergeCandidate(
    const EvaluatedMergeCandidate& candidate,
    const EvaluatedMergeCandidate& current) {
    if (candidate.sim.minClusterMargin !=
        current.sim.minClusterMargin) {
        return candidate.sim.minClusterMargin >
               current.sim.minClusterMargin;
    }
    if (candidate.sim.cooling != current.sim.cooling) {
        return candidate.sim.cooling > current.sim.cooling;
    }
    const int candidateVolume = gridInteriorVolume(candidate.grid);
    const int currentVolume = gridInteriorVolume(current.grid);
    if (candidateVolume != currentVolume) {
        return candidateVolume < currentVolume;
    }
    return countUsefulBlocks(candidate.grid) <
           countUsefulBlocks(current.grid);
}

#ifndef NDEBUG
std::string slotListLabel(const std::vector<int>& slots) {
    std::ostringstream os;
    os << "[";
    for (size_t i = 0; i < slots.size(); ++i) {
        if (i != 0) {
            os << ",";
        }
        os << slots.at(i);
    }
    os << "]";
    return os.str();
}

int unsafeFluxCellCount(const Grid& grid, const FuelSimulation& sim) {
    if (sim.fluxByIndex.size() < static_cast<size_t>(grid.volume()) ||
        sim.functionalCells.size() < static_cast<size_t>(grid.volume())) {
        return 0;
    }

    int count = 0;
    for (const Pos& pos : grid.interiorPositions()) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        const Block& block = grid.atIndex(idx);
        if (block.kind != BlockKind::FuelCell || block.type < 0 ||
            block.type >= static_cast<int>(fuels().size())) {
            continue;
        }
        const Fuel& fuel = fuels().at(static_cast<size_t>(block.type));
        if (sim.fluxByIndex.at(static_cast<size_t>(idx)) > 2.0 * fuel.criticality + 1e-9) {
            ++count;
        }
    }
    return count;
}

bool betterRejectedMergeSnapshot(const MergeRejectionSummary& candidate,
                                 const MergeRejectionSummary& current) {
    if (!current.hasBestRejected) {
        return true;
    }
    if (candidate.bestRejectedRunningCells != current.bestRejectedRunningCells) {
        return candidate.bestRejectedRunningCells > current.bestRejectedRunningCells;
    }
    if (candidate.bestRejectedSafeFlux != current.bestRejectedSafeFlux) {
        return candidate.bestRejectedSafeFlux;
    }
    if ((candidate.bestRejectedDisconnected == 0) != (current.bestRejectedDisconnected == 0)) {
        return candidate.bestRejectedDisconnected == 0;
    }
    if ((candidate.bestRejectedClusters == 1) != (current.bestRejectedClusters == 1)) {
        return candidate.bestRejectedClusters == 1;
    }
    if (candidate.bestRejectedMargin != current.bestRejectedMargin) {
        return candidate.bestRejectedMargin > current.bestRejectedMargin;
    }
    if (candidate.bestRejectedCompatible != current.bestRejectedCompatible) {
        return candidate.bestRejectedCompatible;
    }
    if (candidate.bestRejectedUnsafeFluxCells != current.bestRejectedUnsafeFluxCells) {
        return candidate.bestRejectedUnsafeFluxCells < current.bestRejectedUnsafeFluxCells;
    }
    return candidate.bestRejectedDisconnected < current.bestRejectedDisconnected;
}
#endif

void recordMergeBuildFailure(MergeBuildFailureKind failure, MergeRejectionSummary& summary) {
#ifndef NDEBUG
    switch (failure) {
    case MergeBuildFailureKind::Empty:
        ++summary.buildEmpty;
        NCFR_PERF_COUNT(mergeBuildRejectEmpty);
        break;
    case MergeBuildFailureKind::Size:
        ++summary.buildSize;
        NCFR_PERF_COUNT(mergeBuildRejectSize);
        break;
    case MergeBuildFailureKind::Conflict:
        ++summary.buildConflict;
        NCFR_PERF_COUNT(mergeBuildRejectConflict);
        break;
    case MergeBuildFailureKind::FuelSlot:
        ++summary.buildFuelSlot;
        NCFR_PERF_COUNT(mergeBuildRejectFuelSlot);
        break;
    case MergeBuildFailureKind::FuelDuplicate:
        ++summary.buildFuelDuplicate;
        NCFR_PERF_COUNT(mergeBuildRejectFuelDuplicate);
        break;
    case MergeBuildFailureKind::FuelMissing:
        ++summary.buildFuelMissing;
        NCFR_PERF_COUNT(mergeBuildRejectFuelMissing);
        break;
    case MergeBuildFailureKind::Source:
        ++summary.buildSource;
        NCFR_PERF_COUNT(mergeBuildRejectSource);
        break;
    case MergeBuildFailureKind::None:
        break;
    }
#else
    (void)failure;
    (void)summary;
#endif
}

void recordMergeSimulationRejection(const Grid& grid, const FuelSimulation& sim,
                                    MergeRejectionSummary& summary) {
#ifndef NDEBUG
    const int clusters = heatingClusterCount(sim);
    const bool safeFlux = hasSafeFuelFlux(grid, sim);
    const int unsafeCells = unsafeFluxCellCount(grid, sim);
    const char* reason = "other";
    if (sim.fuelCells <= 0 || sim.runningCells < sim.fuelCells) {
        reason = "notRunnable";
        ++summary.simNotRunnable;
        NCFR_PERF_COUNT(mergeSimulationRejectNotRunnable);
    } else if (!safeFlux) {
        reason = "unsafeFlux";
        ++summary.simUnsafeFlux;
        NCFR_PERF_COUNT(mergeSimulationRejectUnsafeFlux);
    } else if (sim.disconnectedFunctionalBlocks != 0) {
        reason = "disconnected";
        ++summary.simDisconnected;
        NCFR_PERF_COUNT(mergeSimulationRejectDisconnected);
    } else if (!sim.compatible || sim.minClusterMargin < 0) {
        reason = "cooling";
        ++summary.simCooling;
        NCFR_PERF_COUNT(mergeSimulationRejectCooling);
    } else if (clusters != 1) {
        reason = "clusterCount";
        ++summary.simClusterCount;
        NCFR_PERF_COUNT(mergeSimulationRejectClusterCount);
    } else {
        ++summary.simOther;
        NCFR_PERF_COUNT(mergeSimulationRejectOther);
    }

    MergeRejectionSummary candidate;
    candidate.hasBestRejected = true;
    candidate.bestRejectedReason = reason;
    candidate.bestRejectedCompatible = sim.compatible;
    candidate.bestRejectedSafeFlux = safeFlux;
    candidate.bestRejectedFuelCells = sim.fuelCells;
    candidate.bestRejectedRunningCells = sim.runningCells;
    candidate.bestRejectedDisconnected = sim.disconnectedFunctionalBlocks;
    candidate.bestRejectedClusters = clusters;
    candidate.bestRejectedUnsafeFluxCells = unsafeCells;
    candidate.bestRejectedMargin = sim.minClusterMargin;
    candidate.bestRejectedRawHeating = sim.rawHeating;
    candidate.bestRejectedCooling = sim.cooling;
    candidate.bestRejectedA = grid.internalA();
    candidate.bestRejectedB = grid.internalB();
    candidate.bestRejectedC = grid.internalC();
    if (betterRejectedMergeSnapshot(candidate, summary)) {
        summary.hasBestRejected = true;
        summary.bestRejectedReason = candidate.bestRejectedReason;
        summary.bestRejectedCompatible = candidate.bestRejectedCompatible;
        summary.bestRejectedSafeFlux = candidate.bestRejectedSafeFlux;
        summary.bestRejectedFuelCells = candidate.bestRejectedFuelCells;
        summary.bestRejectedRunningCells = candidate.bestRejectedRunningCells;
        summary.bestRejectedDisconnected = candidate.bestRejectedDisconnected;
        summary.bestRejectedClusters = candidate.bestRejectedClusters;
        summary.bestRejectedUnsafeFluxCells = candidate.bestRejectedUnsafeFluxCells;
        summary.bestRejectedMargin = candidate.bestRejectedMargin;
        summary.bestRejectedRawHeating = candidate.bestRejectedRawHeating;
        summary.bestRejectedCooling = candidate.bestRejectedCooling;
        summary.bestRejectedA = candidate.bestRejectedA;
        summary.bestRejectedB = candidate.bestRejectedB;
        summary.bestRejectedC = candidate.bestRejectedC;
    }
#else
    (void)grid;
    (void)sim;
    (void)summary;
#endif
}

void recordMergeAccepted(MergePhase phase, MergeRejectionSummary& summary) {
#ifndef NDEBUG
    ++summary.accepted;
    NCFR_PERF_COUNT(mergeAcceptedCandidates);
    if (phase == MergePhase::Planar) {
        ++summary.acceptedPlanar;
        NCFR_PERF_COUNT(mergeAcceptedPlanarCandidates);
    } else {
        ++summary.acceptedAnyAxis;
        NCFR_PERF_COUNT(mergeAcceptedAnyAxisCandidates);
    }
#else
    (void)phase;
    (void)summary;
#endif
}

void logMergeSummary(const MergeRejectionSummary& summary, const char* result) {
#ifndef NDEBUG
    std::ostringstream os;
    os << "result=" << (result == nullptr ? "unknown" : result)
       << " lhsSlots=" << slotListLabel(summary.lhsSlots)
       << " rhsSlots=" << slotListLabel(summary.rhsSlots)
       << " requestSlots=" << slotListLabel(summary.requestSlots)
       << " lhsSinks=" << summary.lhsSinks
       << " rhsSinks=" << summary.rhsSinks
       << " noHeatingSink=" << summary.noHeatingSink
       << " attempts=" << summary.attempts
       << " planarAttempts=" << summary.planarAttempts
       << " anyAxisAttempts=" << summary.anyAxisAttempts
       << " buildRejects(empty=" << summary.buildEmpty
       << ",size=" << summary.buildSize
       << ",conflict=" << summary.buildConflict
       << ",fuelSlot=" << summary.buildFuelSlot
       << ",fuelDuplicate=" << summary.buildFuelDuplicate
       << ",fuelMissing=" << summary.buildFuelMissing
       << ",source=" << summary.buildSource
       << ") simRejects(notRunnable=" << summary.simNotRunnable
       << ",unsafeFlux=" << summary.simUnsafeFlux
       << ",disconnected=" << summary.simDisconnected
       << ",cooling=" << summary.simCooling
       << ",clusterCount=" << summary.simClusterCount
       << ",other=" << summary.simOther
       << ") accepted=" << summary.accepted
       << " acceptedPlanar=" << summary.acceptedPlanar
       << " acceptedAnyAxis=" << summary.acceptedAnyAxis;
    if (summary.hasBestRejected) {
        os << " bestRejectReason=" << summary.bestRejectedReason
           << " bestRejectGrid=" << summary.bestRejectedA << "x"
           << summary.bestRejectedB << "x" << summary.bestRejectedC
           << " compatible=" << (summary.bestRejectedCompatible ? 1 : 0)
           << " safeFlux=" << (summary.bestRejectedSafeFlux ? 1 : 0)
           << " runningCells=" << summary.bestRejectedRunningCells << "/"
           << summary.bestRejectedFuelCells
           << " minMargin=" << summary.bestRejectedMargin
           << " disconnected=" << summary.bestRejectedDisconnected
           << " clusters=" << summary.bestRejectedClusters
           << " unsafeFluxCells=" << summary.bestRejectedUnsafeFluxCells
           << " rawHeating=" << summary.bestRejectedRawHeating
           << " cooling=" << summary.bestRejectedCooling;
    }
    const std::string checkpoint = os.str();
    NCFR_PERF_CHECKPOINT("merge.summary", checkpoint.c_str());
#else
    (void)summary;
    (void)result;
#endif
}

std::vector<Pos> validHeatingSinkPositions(const Grid& grid) {
    FuelSimulation sim = simulateMixedFuel(grid);
    std::vector<Pos> positions;
    for (const Pos& pos : grid.interiorPositions()) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        const Block& block = grid.atIndex(idx);
        if (block.kind == BlockKind::Sink && block.type >= 0 &&
            sim.validSinks.at(static_cast<size_t>(idx)) &&
            sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            positions.push_back(pos);
        }
    }
    auto boundaryExposure = [&grid](const Pos& pos) {
        int exposure = 0;
        if (pos.x == 1 || pos.x == grid.internalA()) ++exposure;
        if (pos.y == 1 || pos.y == grid.internalB()) ++exposure;
        if (pos.z == 1 || pos.z == grid.internalC()) ++exposure;
        return exposure;
    };
    std::sort(positions.begin(), positions.end(), [&](const Pos& lhs, const Pos& rhs) {
        const int lhsExposure = boundaryExposure(lhs);
        const int rhsExposure = boundaryExposure(rhs);
        if (lhsExposure != rhsExposure) return lhsExposure > rhsExposure;
        if (lhs.z != rhs.z) return lhs.z < rhs.z;
        if (lhs.y != rhs.y) return lhs.y < rhs.y;
        return lhs.x < rhs.x;
    });
    constexpr size_t kMergeSinkCandidateLimit = 96;
    if (positions.size() > kMergeSinkCandidateLimit) {
        positions.resize(kMergeSinkCandidateLimit);
    }
    return positions;
}

struct SubLayout {
    Grid grid;
    std::vector<int> requestSlots;
    std::vector<FuelLayoutContext> fuelContexts;
};

SubLayout optimizeSingleFuelSubLayoutForSlot(const BuildRequest& request, int slot,
                                             const std::atomic_bool* cancelRequested);

struct MergedBlock {
    Pos pos;
    Block block;
    int requestSlot = -1;
};

bool isFixedMergedBlock(const Block& block) {
    return block.kind == BlockKind::FuelCell;
}

std::vector<int> mergedRequestSlots(const SubLayout& lhs, const SubLayout& rhs) {
    std::vector<int> slots = lhs.requestSlots;
    for (int slot : rhs.requestSlots) {
        if (std::find(slots.begin(), slots.end(), slot) == slots.end()) {
            slots.push_back(slot);
        }
    }
    return slots;
}

int requestSlotForFuelBlock(const BuildRequest& request, const SubLayout& layout, const Block& block,
                            std::vector<bool>& usedSlots) {
    for (int slot : layout.requestSlots) {
        if (slot < 0 || slot >= static_cast<int>(request.fuelIndices.size()) ||
            usedSlots.at(static_cast<size_t>(slot))) {
            continue;
        }
        if (request.fuelIndices.at(static_cast<size_t>(slot)) == block.type) {
            usedSlots.at(static_cast<size_t>(slot)) = true;
            return slot;
        }
    }
    return -1;
}

std::vector<MergedBlock> copiedInteriorBlocks(const BuildRequest& request, const SubLayout& layout,
                                              const Pos& translation) {
    std::vector<MergedBlock> blocks;
    blocks.reserve(static_cast<size_t>(layout.grid.internalA() * layout.grid.internalB() * layout.grid.internalC()));
    std::vector<bool> usedFuelSlots(request.fuelIndices.size(), false);
    for (const Pos& pos : layout.grid.interiorPositions()) {
        const Block& block = layout.grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::Empty) {
            continue;
        }
        const int requestSlot = block.kind == BlockKind::FuelCell
            ? requestSlotForFuelBlock(request, layout, block, usedFuelSlots)
            : -1;
        blocks.push_back({{pos.x + translation.x, pos.y + translation.y, pos.z + translation.z},
                          block,
                          requestSlot});
    }
    return blocks;
}

bool openSourceLineToFuel(Grid& grid, const BuildRequest& request, const Pos& sourcePos, const Pos& fuelPos,
                          const Direction& dir) {
    const Block replacement = sourceLineReplacementBlock(request);
    Pos pos = sourcePos;
    while (true) {
        pos.x -= dir.dx;
        pos.y -= dir.dy;
        pos.z -= dir.dz;
        if (!grid.inBounds(pos.x, pos.y, pos.z)) {
            return false;
        }
        if (samePos(pos, fuelPos)) {
            return true;
        }

        Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::FuelCell) {
            return false;
        }
        if (isFullyReflectiveReflector(block)) {
            block = replacement;
        }
    }
}

int requiredSourceCountForSlots(const BuildRequest& request, const std::vector<int>& requestSlots) {
    int count = 0;
    for (int slot : requestSlots) {
        if (slot < 0 || slot >= static_cast<int>(request.fuelIndices.size())) {
            continue;
        }
        const int fuelIndex = request.fuelIndices.at(static_cast<size_t>(slot));
        if (!fuels().at(static_cast<size_t>(fuelIndex)).selfPriming) {
            ++count;
        }
    }
    return count;
}

bool placeMergedSources(Grid& grid, const BuildRequest& request, const std::vector<Pos>& fuelPositions,
                        const std::vector<int>& requestSlots) {
    int placedSources = 0;
    for (int slot : requestSlots) {
        if (slot < 0 || slot >= static_cast<int>(request.fuelIndices.size()) ||
            slot >= static_cast<int>(fuelPositions.size())) {
            return false;
        }
        const Fuel& fuel = fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(slot))));
        if (fuel.selfPriming) {
            continue;
        }

        bool placed = false;
        for (int directionIndex = 0; directionIndex < static_cast<int>(kSourceDirections.size()); ++directionIndex) {
            NCFR_PERF_COUNT(mergeSourcePlacementAttempts);
            const Direction& dir = kSourceDirections.at(static_cast<size_t>(directionIndex));
            const Pos sourcePos = sourcePositionForDirection(grid, fuelPositions.at(static_cast<size_t>(slot)), dir);
            if (!grid.isBoundary(sourcePos.x, sourcePos.y, sourcePos.z) ||
                grid.at(sourcePos.x, sourcePos.y, sourcePos.z).kind != BlockKind::Casing) {
                NCFR_PERF_COUNT(mergeSourceBoundaryRejects);
                continue;
            }

            Grid trial = grid;
            if (!openSourceLineToFuel(trial, request, sourcePos, fuelPositions.at(static_cast<size_t>(slot)), dir)) {
                NCFR_PERF_COUNT(mergeSourceLineRejects);
                continue;
            }
            trial.at(sourcePos.x, sourcePos.y, sourcePos.z) = {BlockKind::Source, -1};
            const int targetIndex = sourcePrimingTargetIndex(trial, sourcePos);
            const int fuelIndex = trial.index(fuelPositions.at(static_cast<size_t>(slot)).x,
                                              fuelPositions.at(static_cast<size_t>(slot)).y,
                                              fuelPositions.at(static_cast<size_t>(slot)).z);
            if (targetIndex == fuelIndex) {
                grid = std::move(trial);
                placed = true;
                NCFR_PERF_COUNT(mergeSourcePlaced);
                break;
            }
            NCFR_PERF_COUNT(mergeSourceTargetRejects);
        }

        if (!placed) {
            return false;
        }
        ++placedSources;
    }
    return placedSources == requiredSourceCountForSlots(request, requestSlots);
}

MergeBuildResult buildMergedGridFromBlocks(const BuildRequest& request,
                                           const std::vector<MergedBlock>& blocks,
                                           const std::vector<int>& requestSlots) {
    if (blocks.empty()) {
        return {{}, {}, MergeBuildFailureKind::Empty};
    }

    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int minZ = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int maxY = std::numeric_limits<int>::min();
    int maxZ = std::numeric_limits<int>::min();
    for (const MergedBlock& block : blocks) {
        minX = std::min(minX, block.pos.x);
        minY = std::min(minY, block.pos.y);
        minZ = std::min(minZ, block.pos.z);
        maxX = std::max(maxX, block.pos.x);
        maxY = std::max(maxY, block.pos.y);
        maxZ = std::max(maxZ, block.pos.z);
    }

    // Keep the uncompressed merge workspace around the copied sublayouts so
    // special cooling can use their boundary-adjacent empty cells before final
    // compaction.
    constexpr int kMergeWorkspacePadding = 1;
    minX -= kMergeWorkspacePadding;
    minY -= kMergeWorkspacePadding;
    minZ -= kMergeWorkspacePadding;
    maxX += kMergeWorkspacePadding;
    maxY += kMergeWorkspacePadding;
    maxZ += kMergeWorkspacePadding;

    const int a = maxX - minX + 1;
    const int b = maxY - minY + 1;
    const int c = maxZ - minZ + 1;
    if (a <= 0 || b <= 0 || c <= 0 || a > kMaxSize || b > kMaxSize || c > kMaxSize) {
        return {{}, {}, MergeBuildFailureKind::Size};
    }

    Grid grid = makeShell(a, b, c);
    std::vector<Pos> fuelPositions(request.fuelIndices.size());
    std::vector<bool> fuelPlaced(request.fuelIndices.size(), false);
    for (const MergedBlock& sourceBlock : blocks) {
        const Pos pos{sourceBlock.pos.x - minX + 1, sourceBlock.pos.y - minY + 1, sourceBlock.pos.z - minZ + 1};
        Block& target = grid.at(pos.x, pos.y, pos.z);
        if (target.kind != BlockKind::Empty) {
            if (isFixedMergedBlock(target) || isFixedMergedBlock(sourceBlock.block) ||
                !sameBlockType(target, sourceBlock.block)) {
                return {{}, {}, MergeBuildFailureKind::Conflict};
            }
            continue;
        }

        target = sourceBlock.block;
        if (sourceBlock.block.kind == BlockKind::FuelCell) {
            if (sourceBlock.requestSlot < 0 ||
                sourceBlock.requestSlot >= static_cast<int>(request.fuelIndices.size())) {
                return {{}, {}, MergeBuildFailureKind::FuelSlot};
            }
            if (request.fuelIndices.at(static_cast<size_t>(sourceBlock.requestSlot)) != sourceBlock.block.type) {
                return {{}, {}, MergeBuildFailureKind::FuelSlot};
            }
            if (fuelPlaced.at(static_cast<size_t>(sourceBlock.requestSlot))) {
                return {{}, {}, MergeBuildFailureKind::FuelDuplicate};
            }
            fuelPositions.at(static_cast<size_t>(sourceBlock.requestSlot)) = pos;
            fuelPlaced.at(static_cast<size_t>(sourceBlock.requestSlot)) = true;
        }
    }

    for (int slot : requestSlots) {
        if (slot < 0 || slot >= static_cast<int>(fuelPlaced.size()) ||
            !fuelPlaced.at(static_cast<size_t>(slot))) {
            return {{}, {}, MergeBuildFailureKind::FuelMissing};
        }
    }
    if (!placeMergedSources(grid, request, fuelPositions, requestSlots)) {
        return {{}, {}, MergeBuildFailureKind::Source};
    }
    MergeBuildResult result;
    result.grid = std::move(grid);
    result.fuelPositions = std::move(fuelPositions);
    return result;
}

std::vector<FuelLayoutContext> mergedFuelContexts(
    const SubLayout& lhs, const SubLayout& rhs,
    const std::vector<Pos>& fuelPositions) {
    std::vector<FuelLayoutContext> contexts = lhs.fuelContexts;
    contexts.insert(
        contexts.end(), rhs.fuelContexts.begin(), rhs.fuelContexts.end());
    for (FuelLayoutContext& context : contexts) {
        if (context.requestSlot >= 0 &&
            context.requestSlot < static_cast<int>(fuelPositions.size())) {
            context.fuelPos =
                fuelPositions.at(static_cast<size_t>(context.requestSlot));
        }
    }
    return contexts;
}

std::optional<EvaluatedMergeCandidate> tryMergeCandidatesForPhase(
    const BuildRequest& request, const SubLayout& lhs, const SubLayout& rhs,
    const std::vector<Pos>& lhsSinks, const std::vector<Pos>& rhsSinks,
    const std::vector<int>& requestSlots, MergePhase phase,
    MergeFallbackPolicy fallbackPolicy,
    MergeRejectionSummary& summary, const std::atomic_bool* cancelRequested) {
    std::optional<EvaluatedMergeCandidate> bestPlanarMerge;
    std::optional<MergeCandidateScore> bestPlanarScore;
    std::optional<EvaluatedMergeCandidate> bestCoolingDeficit;
    const bool allowCoolingDeficit =
        fallbackPolicy == MergeFallbackPolicy::SpecialCooling &&
        request.fuelIndices.size() == 2 &&
        requestSlots.size() == request.fuelIndices.size();
    for (const Pos& lhsSink : lhsSinks) {
        for (const Pos& rhsSink : rhsSinks) {
            for (const Direction& dir : kSourceDirections) {
                throwIfCancelled(cancelRequested);
                const Pos rhsTranslation{lhsSink.x + dir.dx - rhsSink.x,
                                         lhsSink.y + dir.dy - rhsSink.y,
                                         lhsSink.z + dir.dz - rhsSink.z};
                if (phase == MergePhase::Planar && (dir.dz != 0 || rhsTranslation.z != 0)) {
                    continue;
                }
#ifndef NDEBUG
                ++summary.attempts;
#endif
                NCFR_PERF_COUNT(mergeCandidateAttempts);
                if (phase == MergePhase::Planar) {
#ifndef NDEBUG
                    ++summary.planarAttempts;
#endif
                    NCFR_PERF_COUNT(mergePlanarCandidateAttempts);
                } else {
#ifndef NDEBUG
                    ++summary.anyAxisAttempts;
#endif
                    NCFR_PERF_COUNT(mergeAnyAxisCandidateAttempts);
                }
                std::vector<MergedBlock> blocks = copiedInteriorBlocks(request, lhs, {0, 0, 0});
                std::vector<MergedBlock> rhsBlocks = copiedInteriorBlocks(request, rhs, rhsTranslation);
                blocks.insert(blocks.end(), rhsBlocks.begin(), rhsBlocks.end());

                MergeBuildResult merged = buildMergedGridFromBlocks(request, blocks, requestSlots);
                if (!merged.grid.has_value()) {
                    recordMergeBuildFailure(merged.failure, summary);
                    continue;
                }
                FuelSimulation sim = simulateMixedFuel(*merged.grid);
                std::vector<FuelLayoutContext> fuelContexts =
                    mergedFuelContexts(
                        lhs, rhs, merged.fuelPositions);
                if (isSearchAccepted(*merged.grid, sim) && heatingClusterCount(sim) == 1) {
                    Grid acceptedGrid = std::move(*merged.grid);
                    FuelSimulation acceptedSim = std::move(sim);
                    const bool finalMerge =
                        requestSlots.size() == request.fuelIndices.size();
                    if (finalMerge) {
#ifndef NDEBUG
                        const int oldA = acceptedGrid.internalA();
                        const int oldB = acceptedGrid.internalB();
                        const int oldC = acceptedGrid.internalC();
#endif
                        acceptedGrid =
                            compactEmptyInteriorPlanes(std::move(acceptedGrid));
                        acceptedSim = simulateMixedFuel(acceptedGrid);
                        const WallConnectionResult wall =
                            evaluateHeatingClusterWallConnections(acceptedGrid,
                                                                    acceptedSim);
#ifndef NDEBUG
                        {
                            std::ostringstream detail;
                            detail << "mode=merge old=" << oldA << "x" << oldB
                                   << "x" << oldC
                                   << " new=" << gridInteriorLabel(acceptedGrid)
                                   << " heatingClusters=" << wall.heatingClusters
                                   << " wallDisconnected="
                                   << wall.disconnectedHeatingClusters
                                   << " sourcesValid="
                                   << (hasRequiredSources(acceptedGrid, request)
                                           ? 1
                                           : 0);
                            NCFR_PERF_CHECKPOINT("wallConnection.final",
                                                 detail.str().c_str());
                        }
#endif
                        if (!isSearchAccepted(acceptedGrid, acceptedSim) ||
                            hasInvalidSinks(acceptedGrid, acceptedSim) ||
                            heatingClusterCount(acceptedSim) != 1 ||
                            !wall.allConnected() ||
                            !hasRequiredSources(acceptedGrid, request)) {
                            recordMergeSimulationRejection(acceptedGrid,
                                                           acceptedSim, summary);
                            continue;
                        }
                    }
                    recordMergeAccepted(phase, summary);
                    if (phase == MergePhase::Planar) {
                        const MergeCandidateScore score =
                            mergeCandidateScore(acceptedGrid, acceptedSim);
                        if (!bestPlanarScore.has_value() || isBetterMergeCandidate(score, *bestPlanarScore)) {
                            NCFR_PERF_COUNT(bestUpdates);
                            bestPlanarScore = score;
                            EvaluatedMergeCandidate candidate{
                                std::move(acceptedGrid),
                                std::move(acceptedSim),
                                std::move(fuelContexts)};
                            bestPlanarMerge = std::move(candidate);
                        }
                    } else {
                        NCFR_PERF_COUNT(bestUpdates);
                        EvaluatedMergeCandidate candidate{
                            std::move(acceptedGrid),
                            std::move(acceptedSim),
                            std::move(fuelContexts)};
                        return candidate;
                    }
                    continue;
                }
                if (allowCoolingDeficit &&
                    sim.minClusterMargin < 0 &&
                    isStructurallyMergeable(*merged.grid, sim)) {
                    EvaluatedMergeCandidate candidate{
                        std::move(*merged.grid),
                        std::move(sim),
                        std::move(fuelContexts),
                    };
                    if (!bestCoolingDeficit.has_value() ||
                        betterCoolingDeficitMergeCandidate(
                            candidate, *bestCoolingDeficit)) {
                        NCFR_PERF_COUNT(bestUpdates);
                        bestCoolingDeficit = std::move(candidate);
                    }
                    continue;
                }
                recordMergeSimulationRejection(*merged.grid, sim, summary);
            }
        }
    }
    if (bestPlanarMerge.has_value()) {
        return bestPlanarMerge;
    }
    return bestCoolingDeficit;
}

std::optional<EvaluatedMergeCandidate> finalizeMergedCandidate(
    EvaluatedMergeCandidate candidate, const BuildRequest& request,
    const std::vector<int>& requestSlots,
    MergeFallbackPolicy fallbackPolicy,
    const std::atomic_bool* cancelRequested) {
    if (request.fuelIndices.size() != 2) {
        return isSearchAccepted(candidate.grid, candidate.sim)
                   ? std::optional<EvaluatedMergeCandidate>(
                         std::move(candidate))
                   : std::nullopt;
    }

    const bool finalMerge =
        requestSlots.size() == request.fuelIndices.size();
    if (!finalMerge) {
        return isSearchAccepted(candidate.grid, candidate.sim)
                   ? std::optional<EvaluatedMergeCandidate>(
                         std::move(candidate))
                   : std::nullopt;
    }

    if (!isSearchAccepted(candidate.grid, candidate.sim)) {
        if (fallbackPolicy != MergeFallbackPolicy::SpecialCooling) {
            return std::nullopt;
        }
        if (!isStructurallyMergeable(
                candidate.grid, candidate.sim)) {
            return std::nullopt;
        }

        std::optional<Grid> special =
            tryMixedFuelSpecialCoolingFallback(
                candidate.grid, request,
                candidate.fuelContexts, cancelRequested);
        if (!special.has_value()) {
            return std::nullopt;
        }
        candidate.grid = std::move(*special);
        candidate.sim = simulateMixedFuel(candidate.grid);
    }

    candidate.grid =
        compactEmptyInteriorPlanes(std::move(candidate.grid));
    candidate.sim = simulateMixedFuel(candidate.grid);
    const WallConnectionResult wall =
        evaluateHeatingClusterWallConnections(
            candidate.grid, candidate.sim);
    if (!isSearchAccepted(candidate.grid, candidate.sim) ||
        hasInvalidSinks(candidate.grid, candidate.sim) ||
        heatingClusterCount(candidate.sim) != 1 ||
        !wall.allConnected() ||
        !hasRequiredSources(candidate.grid, request)) {
        return std::nullopt;
    }
    return candidate;
}

std::optional<EvaluatedMergeCandidate> tryMergeLayoutGrids(
    const BuildRequest& request, const SubLayout& lhs,
    const SubLayout& rhs,
    MergeFallbackPolicy fallbackPolicy,
    const std::atomic_bool* cancelRequested) {
    NCFR_PERF_COUNT(mergeLayoutCalls);
    MergeRejectionSummary summary;
    const std::vector<int> requestSlots = mergedRequestSlots(lhs, rhs);
#ifndef NDEBUG
    summary.lhsSlots = lhs.requestSlots;
    summary.rhsSlots = rhs.requestSlots;
    summary.requestSlots = requestSlots;
#endif

    const std::vector<Pos> lhsSinks = validHeatingSinkPositions(lhs.grid);
    const std::vector<Pos> rhsSinks = validHeatingSinkPositions(rhs.grid);
#ifndef NDEBUG
    summary.lhsSinks = lhsSinks.size();
    summary.rhsSinks = rhsSinks.size();
#endif
    if (lhsSinks.empty() || rhsSinks.empty()) {
#ifndef NDEBUG
        ++summary.noHeatingSink;
#endif
        NCFR_PERF_COUNT(mergeNoHeatingSinkRejects);
        logMergeSummary(summary, "noHeatingSink");
        return std::nullopt;
    }

    if (std::optional<EvaluatedMergeCandidate> planar = tryMergeCandidatesForPhase(
            request, lhs, rhs, lhsSinks, rhsSinks, requestSlots, MergePhase::Planar,
            fallbackPolicy, summary, cancelRequested)) {
        if (std::optional<EvaluatedMergeCandidate> finalized =
                finalizeMergedCandidate(
                    std::move(*planar), request, requestSlots,
                    fallbackPolicy, cancelRequested)) {
            logMergeSummary(summary, "acceptedPlanar");
            return finalized;
        }
    }

    if (std::optional<EvaluatedMergeCandidate> anyAxis = tryMergeCandidatesForPhase(
            request, lhs, rhs, lhsSinks, rhsSinks, requestSlots, MergePhase::AnyAxis,
            fallbackPolicy, summary, cancelRequested)) {
        if (std::optional<EvaluatedMergeCandidate> finalized =
                finalizeMergedCandidate(
                    std::move(*anyAxis), request, requestSlots,
                    fallbackPolicy, cancelRequested)) {
            logMergeSummary(summary, "acceptedAnyAxis");
            return finalized;
        }
    }

    logMergeSummary(summary, "rejected");
    return std::nullopt;
}

std::optional<OptimizationResult> tryMergeDualLayouts(
    const BuildRequest& request, const SubLayout& lhs, const SubLayout& rhs,
    MergeFallbackPolicy fallbackPolicy,
    const std::atomic_bool* cancelRequested) {
    std::optional<EvaluatedMergeCandidate> merged =
        tryMergeLayoutGrids(
            request, lhs, rhs, fallbackPolicy, cancelRequested);
    if (!merged.has_value()) {
        return std::nullopt;
    }

    return resultFromSimulation(
        std::move(merged->grid), request, merged->sim);
}

OptimizationResult optimizeDualFuelLayout(
    const BuildRequest& request, const std::atomic_bool* cancelRequested) {
    if (request.fuelIndices.size() != 2) {
        throw std::invalid_argument("双燃料策略需要 2 个燃料单元。");
    }

    const Fuel& firstFuel = fuels().at(
        static_cast<size_t>(request.fuelIndices.at(0)));
    const Fuel& secondFuel = fuels().at(
        static_cast<size_t>(request.fuelIndices.at(1)));
    const int highSlot = firstFuel.heat >= secondFuel.heat ? 0 : 1;
    const int lowSlot = 1 - highSlot;

    const MergeableSingleFuelSearchGoal highGoal{
        kDualFuelStageCoolingTarget,
        0,
        0,
        false,
    };
    MergeableSingleFuelLayout highLayout =
        optimizeMergeableSingleFuelForSlot(
            request, highSlot, highGoal, cancelRequested);
    const MergeableSingleFuelSearchGoal lowGoal{
        kDualFuelStageCoolingTarget,
        highLayout.sim.rawHeating,
        highLayout.sim.cooling,
        true,
    };
    MergeableSingleFuelLayout lowLayout =
        optimizeMergeableSingleFuelForSlot(
            request, lowSlot, lowGoal, cancelRequested);

    const long long combinedCooling =
        highLayout.sim.cooling + lowLayout.sim.cooling;
    const long long combinedRawHeating =
        highLayout.sim.rawHeating + lowLayout.sim.rawHeating;
    const MergeFallbackPolicy fallbackPolicy =
        combinedCooling >= combinedRawHeating
            ? MergeFallbackPolicy::Disabled
            : MergeFallbackPolicy::SpecialCooling;

    const std::vector<Pos> highFuelPositions =
        fuelPositionsInGrid(highLayout.grid);
    const std::vector<Pos> lowFuelPositions =
        fuelPositionsInGrid(lowLayout.grid);
    if (highFuelPositions.size() != 1 ||
        lowFuelPositions.size() != 1) {
        throw std::runtime_error(
            "双燃料单独生成结果中的燃料单元数量异常。");
    }
    SubLayout high{
        std::move(highLayout.grid),
        {highSlot},
        {{highSlot, highFuelPositions.front(),
          std::move(highLayout.sourceDirections),
          std::move(highLayout.fuelLines)}}};
    SubLayout low{
        std::move(lowLayout.grid),
        {lowSlot},
        {{lowSlot, lowFuelPositions.front(),
          std::move(lowLayout.sourceDirections),
          std::move(lowLayout.fuelLines)}}};
    if (std::optional<OptimizationResult> merged =
            tryMergeDualLayouts(
                request, high, low, fallbackPolicy, cancelRequested);
        merged.has_value()) {
        return std::move(*merged);
    }

    throw std::runtime_error("无满足双燃料输入要求的合并方案。");
}

SubLayout optimizeSingleFuelSubLayoutForSlot(const BuildRequest& request, int slot,
                                             const std::atomic_bool* cancelRequested) {
    OptimizationResult result = optimizeSingleFuelForSlot(request, slot, cancelRequested);
    return {std::move(result.grid), {slot}, {}};
}

std::optional<Grid> tryMergeLayoutsInOrder(const BuildRequest& request, const std::vector<SubLayout>& layouts,
                                           const std::vector<int>& order,
                                           const std::atomic_bool* cancelRequested) {
    if (order.empty()) {
        return std::nullopt;
    }

    SubLayout merged = layouts.at(static_cast<size_t>(order.front()));
    for (size_t index = 1; index < order.size(); ++index) {
        throwIfCancelled(cancelRequested);
        const SubLayout& next = layouts.at(static_cast<size_t>(order.at(index)));
        const std::vector<int> requestSlots = mergedRequestSlots(merged, next);
        std::optional<EvaluatedMergeCandidate> mergedCandidate =
            tryMergeLayoutGrids(
                request, merged, next, MergeFallbackPolicy::Disabled,
                cancelRequested);
        if (!mergedCandidate.has_value()) {
            return std::nullopt;
        }
        merged = {
            std::move(mergedCandidate->grid),
            requestSlots,
            std::move(mergedCandidate->fuelContexts)};
    }

    FuelSimulation sim = simulateMixedFuel(merged.grid);
    if (!isSearchAccepted(merged.grid, sim) || heatingClusterCount(sim) != 1) {
        return std::nullopt;
    }
    return std::move(merged.grid);
}

OptimizationResult optimizeQuadFuelLayout(const BuildRequest& request, const std::atomic_bool* cancelRequested) {
    if (request.fuelIndices.size() != 4) {
        throw std::invalid_argument("四燃料策略需要 4 个燃料单元。");
    }

    std::vector<SubLayout> layouts;
    layouts.reserve(request.fuelIndices.size());
    for (int slot = 0; slot < static_cast<int>(request.fuelIndices.size()); ++slot) {
        throwIfCancelled(cancelRequested);
        layouts.push_back(optimizeSingleFuelSubLayoutForSlot(request, slot, cancelRequested));
    }

    std::vector<int> order{0, 1, 2, 3};
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        return heatPriorityLess(lhs, rhs, request);
    });
    std::vector<std::vector<int>> orders{order};

    order = {0, 1, 2, 3};
    do {
        if (std::find(orders.begin(), orders.end(), order) == orders.end()) {
            orders.push_back(order);
        }
    } while (std::next_permutation(order.begin(), order.end()));

    for (const std::vector<int>& candidateOrder : orders) {
        throwIfCancelled(cancelRequested);
        std::optional<Grid> merged = tryMergeLayoutsInOrder(request, layouts, candidateOrder, cancelRequested);
        if (merged.has_value()) {
            FuelSimulation sim = simulateMixedFuel(*merged);
            return resultFromSimulation(std::move(*merged), request, sim);
        }
    }

    throw std::runtime_error("无满足四燃料输入要求的合并方案。");
}

} // namespace ncfr::optimizer_detail
