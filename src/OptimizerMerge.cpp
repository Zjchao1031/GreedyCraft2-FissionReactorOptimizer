#include "OptimizerDetail.h"

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

Grid cloneSingleFuelGridAs(const Grid& source, int fuelIndex) {
    Grid grid = source;
    for (const Pos& pos : grid.interiorPositions()) {
        Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::FuelCell) {
            block.type = fuelIndex;
        }
    }
    return grid;
}

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
};

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

bool openSourceLineToFuel(Grid& grid, const Pos& sourcePos, const Pos& fuelPos, const Direction& dir) {
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
            block = {BlockKind::Reflector, 1};
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
            const Direction& dir = kSourceDirections.at(static_cast<size_t>(directionIndex));
            const Pos sourcePos = sourcePositionForDirection(grid, fuelPositions.at(static_cast<size_t>(slot)), dir);
            if (!grid.isBoundary(sourcePos.x, sourcePos.y, sourcePos.z) ||
                grid.at(sourcePos.x, sourcePos.y, sourcePos.z).kind != BlockKind::Casing) {
                continue;
            }

            Grid trial = grid;
            if (!openSourceLineToFuel(trial, sourcePos, fuelPositions.at(static_cast<size_t>(slot)), dir)) {
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
                break;
            }
        }

        if (!placed) {
            return false;
        }
        ++placedSources;
    }
    return placedSources == requiredSourceCountForSlots(request, requestSlots);
}

std::optional<Grid> buildMergedGridFromBlocks(const BuildRequest& request,
                                              const std::vector<MergedBlock>& blocks,
                                              const std::vector<int>& requestSlots) {
    if (blocks.empty()) {
        return std::nullopt;
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

    const int a = maxX - minX + 1;
    const int b = maxY - minY + 1;
    const int c = maxZ - minZ + 1;
    if (a <= 0 || b <= 0 || c <= 0 || a > kMaxSize || b > kMaxSize || c > kMaxSize) {
        return std::nullopt;
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
                return std::nullopt;
            }
            continue;
        }

        target = sourceBlock.block;
        if (sourceBlock.block.kind == BlockKind::FuelCell) {
            if (sourceBlock.requestSlot < 0 ||
                sourceBlock.requestSlot >= static_cast<int>(request.fuelIndices.size())) {
                return std::nullopt;
            }
            if (request.fuelIndices.at(static_cast<size_t>(sourceBlock.requestSlot)) != sourceBlock.block.type ||
                fuelPlaced.at(static_cast<size_t>(sourceBlock.requestSlot))) {
                return std::nullopt;
            }
            fuelPositions.at(static_cast<size_t>(sourceBlock.requestSlot)) = pos;
            fuelPlaced.at(static_cast<size_t>(sourceBlock.requestSlot)) = true;
        }
    }

    for (int slot : requestSlots) {
        if (slot < 0 || slot >= static_cast<int>(fuelPlaced.size()) ||
            !fuelPlaced.at(static_cast<size_t>(slot))) {
            return std::nullopt;
        }
    }
    if (!placeMergedSources(grid, request, fuelPositions, requestSlots)) {
        return std::nullopt;
    }
    return grid;
}

std::optional<Grid> tryMergeLayoutGrids(const BuildRequest& request, const SubLayout& lhs,
                                        const SubLayout& rhs,
                                        const std::atomic_bool* cancelRequested) {
    const std::vector<Pos> lhsSinks = validHeatingSinkPositions(lhs.grid);
    const std::vector<Pos> rhsSinks = validHeatingSinkPositions(rhs.grid);
    if (lhsSinks.empty() || rhsSinks.empty()) {
        return std::nullopt;
    }

    const std::vector<int> requestSlots = mergedRequestSlots(lhs, rhs);
    std::optional<Grid> bestPlanarMerge;
    std::optional<MergeCandidateScore> bestPlanarScore;

    for (const Pos& lhsSink : lhsSinks) {
        for (const Pos& rhsSink : rhsSinks) {
            for (const Direction& dir : kSourceDirections) {
                throwIfCancelled(cancelRequested);
                const Pos rhsTranslation{lhsSink.x + dir.dx - rhsSink.x,
                                         lhsSink.y + dir.dy - rhsSink.y,
                                         lhsSink.z + dir.dz - rhsSink.z};
                if (dir.dz != 0 || rhsTranslation.z != 0) {
                    continue;
                }
                std::vector<MergedBlock> blocks = copiedInteriorBlocks(request, lhs, {0, 0, 0});
                std::vector<MergedBlock> rhsBlocks = copiedInteriorBlocks(request, rhs, rhsTranslation);
                blocks.insert(blocks.end(), rhsBlocks.begin(), rhsBlocks.end());

                std::optional<Grid> merged = buildMergedGridFromBlocks(request, blocks, requestSlots);
                if (!merged.has_value()) {
                    continue;
                }
                FuelSimulation sim = simulateMixedFuel(*merged);
                if (isAccepted(*merged, sim) && heatingClusterCount(sim) == 1) {
                    const MergeCandidateScore score = mergeCandidateScore(*merged, sim);
                    if (!bestPlanarScore.has_value() || isBetterMergeCandidate(score, *bestPlanarScore)) {
                        NCFR_PERF_COUNT(bestUpdates);
                        bestPlanarScore = score;
                        bestPlanarMerge = std::move(merged);
                    }
                }
            }
        }
    }

    if (bestPlanarMerge.has_value()) {
        return bestPlanarMerge;
    }

    for (const Pos& lhsSink : lhsSinks) {
        for (const Pos& rhsSink : rhsSinks) {
            for (const Direction& dir : kSourceDirections) {
                throwIfCancelled(cancelRequested);
                const Pos rhsTranslation{lhsSink.x + dir.dx - rhsSink.x,
                                         lhsSink.y + dir.dy - rhsSink.y,
                                         lhsSink.z + dir.dz - rhsSink.z};
                std::vector<MergedBlock> blocks = copiedInteriorBlocks(request, lhs, {0, 0, 0});
                std::vector<MergedBlock> rhsBlocks = copiedInteriorBlocks(request, rhs, rhsTranslation);
                blocks.insert(blocks.end(), rhsBlocks.begin(), rhsBlocks.end());

                std::optional<Grid> merged = buildMergedGridFromBlocks(request, blocks, requestSlots);
                if (!merged.has_value()) {
                    continue;
                }
                FuelSimulation sim = simulateMixedFuel(*merged);
                if (isAccepted(*merged, sim) && heatingClusterCount(sim) == 1) {
                    NCFR_PERF_COUNT(bestUpdates);
                    return merged;
                }
            }
        }
    }
    return std::nullopt;
}

std::optional<OptimizationResult> tryMergeDualLayouts(const BuildRequest& request, const SubLayout& lhs,
                                                      const SubLayout& rhs,
                                                      const std::atomic_bool* cancelRequested) {
    std::optional<Grid> merged = tryMergeLayoutGrids(request, lhs, rhs, cancelRequested);
    if (!merged.has_value()) {
        return std::nullopt;
    }

    FuelSimulation sim = simulateMixedFuel(*merged);
    return resultFromSimulation(std::move(*merged), request, sim);
}

OptimizationResult optimizeDualFuelLayout(const BuildRequest& request, const std::atomic_bool* cancelRequested) {
    const bool sameStartupType =
        fuels().at(static_cast<size_t>(request.fuelIndices.at(0))).selfPriming ==
        fuels().at(static_cast<size_t>(request.fuelIndices.at(1))).selfPriming;

    if (sameStartupType) {
        const int primarySlot = primarySameStartupSlot(request);
        const int secondarySlot = 1 - primarySlot;
        OptimizationResult primaryResult = optimizeSingleFuelForSlot(request, primarySlot, cancelRequested);
        SubLayout primary{primaryResult.grid, {primarySlot}};
        SubLayout secondary{cloneSingleFuelGridAs(primaryResult.grid,
                                                  request.fuelIndices.at(static_cast<size_t>(secondarySlot))),
                            {secondarySlot}};
        const FuelSimulation clonedSecondarySim = simulateMixedFuel(secondary.grid);
        if (isAccepted(secondary.grid, clonedSecondarySim)) {
            if (std::optional<OptimizationResult> merged =
                    tryMergeDualLayouts(request, primary, secondary, cancelRequested);
                merged.has_value()) {
                return std::move(*merged);
            }
        }

        OptimizationResult secondaryResult = optimizeSingleFuelForSlot(request, secondarySlot, cancelRequested);
        secondary = {std::move(secondaryResult.grid), {secondarySlot}};
        if (std::optional<OptimizationResult> merged =
                tryMergeDualLayouts(request, primary, secondary, cancelRequested);
            merged.has_value()) {
            return std::move(*merged);
        }
    } else {
        OptimizationResult firstResult = optimizeSingleFuelForSlot(request, 0, cancelRequested);
        OptimizationResult secondResult = optimizeSingleFuelForSlot(request, 1, cancelRequested);
        SubLayout first{std::move(firstResult.grid), {0}};
        SubLayout second{std::move(secondResult.grid), {1}};
        if (std::optional<OptimizationResult> merged =
                tryMergeDualLayouts(request, first, second, cancelRequested);
            merged.has_value()) {
            return std::move(*merged);
        }
    }

    throw std::runtime_error("无满足双燃料输入要求的合并方案。");
}

SubLayout optimizeSingleFuelSubLayoutForSlot(const BuildRequest& request, int slot,
                                             const std::atomic_bool* cancelRequested) {
    OptimizationResult result = optimizeSingleFuelForSlot(request, slot, cancelRequested);
    return {std::move(result.grid), {slot}};
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
        std::optional<Grid> mergedGrid = tryMergeLayoutGrids(request, merged, next, cancelRequested);
        if (!mergedGrid.has_value()) {
            return std::nullopt;
        }
        merged = {std::move(*mergedGrid), requestSlots};
    }

    FuelSimulation sim = simulateMixedFuel(merged.grid);
    if (!isAccepted(merged.grid, sim) || heatingClusterCount(sim) != 1) {
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
