#include "OptimizerDetail.h"

#include "NeutronRules.h"
#include "Perf.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ncfr::optimizer_detail {
namespace {

constexpr double kFluxEpsilon = 1e-9;
constexpr int kFuelLineDirection = 0;
constexpr int kSourceDirection = 1;
constexpr long long kManaDustFallbackCapacity = 640;

enum class DualTemplateMode {
    Aligned,
    Unequal,
};

struct DualTemplateSpec {
    std::array<FuelLineSpec, 2> lines;
    DualTemplateMode mode = DualTemplateMode::Aligned;
};

struct BuiltDualFuelSkeleton {
    Grid grid;
    StateVector protectedPositions;
    std::array<Pos, 2> fuelPositions;
    std::array<Pos, 2> reflectorPositions;
    std::vector<Pos> fixedSinkPositions;
    std::vector<Pos> endStoneCandidates;
    DualTemplateSpec spec;
};

struct PaddedDualGrid {
    Grid grid;
    StateVector protectedPositions;
};

Pos indexToPos(const Grid& grid, int idx) {
    return {
        idx % grid.width(),
        (idx / grid.width()) % grid.height(),
        idx / (grid.width() * grid.height()),
    };
}

void markProtected(StateVector& protectedPositions, const Grid& grid, const Pos& pos) {
    if (grid.inBounds(pos.x, pos.y, pos.z)) {
        protectedPositions.at(static_cast<size_t>(grid.index(pos.x, pos.y, pos.z))) = true;
    }
}

int sinkTypeForSourceName(const char* sourceName) {
    for (const SinkType& sink : sinkTypes()) {
        if (sink.sourceName == sourceName) {
            return sink.index;
        }
    }
    return -1;
}

int manganeseSinkType() {
    static const int type = sinkTypeForSourceName("manganese");
    return type;
}

int glowstoneSinkType() {
    static const int type = sinkTypeForSourceName("glowstone");
    return type;
}

int arsenicSinkType() {
    static const int type = sinkTypeForSourceName("arsenic");
    return type;
}

int endStoneSinkType() {
    static const int type = sinkTypeForSourceName("end_stone");
    return type;
}

bool sameBlock(const Block& lhs, const Block& rhs) {
    return lhs.kind == rhs.kind && lhs.type == rhs.type;
}

std::vector<Pos> orderedFuelPositions(const Grid& grid) {
    std::vector<Pos> positions = fuelPositionsInGrid(grid);
    std::sort(positions.begin(), positions.end(), [](const Pos& lhs, const Pos& rhs) {
        if (lhs.y != rhs.y) return lhs.y < rhs.y;
        if (lhs.x != rhs.x) return lhs.x < rhs.x;
        return lhs.z < rhs.z;
    });
    return positions;
}

std::vector<Pos> interiorCornerPositions(const Grid& grid) {
    return {
        {1, 1, 1},
        {grid.internalA(), 1, 1},
        {1, grid.internalB(), 1},
        {grid.internalA(), grid.internalB(), 1},
        {1, 1, grid.internalC()},
        {grid.internalA(), 1, grid.internalC()},
        {1, grid.internalB(), grid.internalC()},
        {grid.internalA(), grid.internalB(), grid.internalC()},
    };
}

bool placeDualSources(Grid& grid, const BuildRequest& request,
                      const std::array<Pos, 2>& fuelPositions,
                      StateVector& protectedPositions) {
    const Direction& sourceDirection =
        kSourceDirections.at(static_cast<size_t>(kSourceDirection));
    int placedSources = 0;
    for (int slot = 0; slot < 2; ++slot) {
        const Fuel& fuel =
            fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(slot))));
        if (fuel.selfPriming) {
            continue;
        }

        const Pos fuelPos = fuelPositions.at(static_cast<size_t>(slot));
        const Pos sourcePos = sourcePositionForDirection(grid, fuelPos, sourceDirection);
        if (!grid.isBoundary(sourcePos.x, sourcePos.y, sourcePos.z) ||
            grid.at(sourcePos.x, sourcePos.y, sourcePos.z).kind != BlockKind::Casing) {
            return false;
        }

        for (int x = 1; x < fuelPos.x; ++x) {
            const Pos pathPos{x, fuelPos.y, fuelPos.z};
            grid.at(pathPos.x, pathPos.y, pathPos.z) = {BlockKind::Empty, -1};
        }
        grid.at(sourcePos.x, sourcePos.y, sourcePos.z) = {BlockKind::Source, -1};
        markProtected(protectedPositions, grid, sourcePos);
        if (sourcePrimingTargetIndex(grid, sourcePos) !=
            grid.index(fuelPos.x, fuelPos.y, fuelPos.z)) {
            return false;
        }
        ++placedSources;
    }
    return placedSources == requiredSourceCountForFuels(request);
}

std::vector<Pos> endStoneCandidatePositions(
    const Grid& grid, const std::array<Pos, 2>& reflectorPositions) {
    std::vector<Pos> candidates;
    for (const Pos& reflectorPos : reflectorPositions) {
        for (int directionIndex = 0;
             directionIndex < static_cast<int>(kSourceDirections.size());
             ++directionIndex) {
            if (directionIndex == kSourceDirection) {
                continue;
            }
            const Pos candidate =
                offset(reflectorPos,
                       kSourceDirections.at(static_cast<size_t>(directionIndex)), 1);
            if (!grid.isInterior(candidate.x, candidate.y, candidate.z) ||
                grid.at(candidate.x, candidate.y, candidate.z).kind !=
                    BlockKind::Empty) {
                continue;
            }
            if (std::none_of(candidates.begin(), candidates.end(),
                             [&](const Pos& existing) {
                    return samePos(existing, candidate);
                })) {
                candidates.push_back(candidate);
            }
        }
    }
    return candidates;
}

std::optional<BuiltDualFuelSkeleton> buildDualFuelSkeleton(
    const BuildRequest& request, const DualTemplateSpec& spec) {
    const int manganeseType = manganeseSinkType();
    const int glowstoneType = glowstoneSinkType();
    const int arsenicType = arsenicSinkType();
    if (manganeseType < 0 || glowstoneType < 0 ||
        (spec.mode == DualTemplateMode::Aligned && arsenicType < 0)) {
        return std::nullopt;
    }

    Grid grid = makeShell(kMaxSize, kMaxSize, kMaxSize);
    StateVector protectedPositions(static_cast<size_t>(grid.volume()), false);
    const int center = (kMaxSize + 1) / 2;
    const int fuelX = center;
    const int middleY = center;
    const int fuelZ = center;
    const std::array<Pos, 2> fuelPositions{{
        {fuelX, middleY - 1, fuelZ},
        {fuelX, middleY + 1, fuelZ},
    }};
    std::array<Pos, 2> reflectorPositions{};
    std::vector<Pos> fixedSinkPositions;

    for (int slot = 0; slot < 2; ++slot) {
        const FuelLineSpec& line = spec.lines.at(static_cast<size_t>(slot));
        if (line.direction != kFuelLineDirection ||
            line.moderatorCount < 1 ||
            line.moderatorCount > kMaxReflectorLineModerators) {
            return std::nullopt;
        }

        const Pos fuelPos = fuelPositions.at(static_cast<size_t>(slot));
        grid.at(fuelPos.x, fuelPos.y, fuelPos.z) = {
            BlockKind::FuelCell,
            request.fuelIndices.at(static_cast<size_t>(slot)),
        };
        markProtected(protectedPositions, grid, fuelPos);
        for (int distance = 1; distance <= line.moderatorCount; ++distance) {
            const Pos moderatorPos{fuelPos.x + distance, fuelPos.y, fuelPos.z};
            grid.at(moderatorPos.x, moderatorPos.y, moderatorPos.z) = {
                BlockKind::Moderator,
                line.moderatorType,
            };
            markProtected(protectedPositions, grid, moderatorPos);
        }
        const Pos reflectorPos{
            fuelPos.x + line.moderatorCount + 1,
            fuelPos.y,
            fuelPos.z,
        };
        grid.at(reflectorPos.x, reflectorPos.y, reflectorPos.z) = {
            BlockKind::Reflector,
            line.reflectorType,
        };
        markProtected(protectedPositions, grid, reflectorPos);
        reflectorPositions.at(static_cast<size_t>(slot)) = reflectorPos;
    }

    const Pos manganesePos{fuelX, middleY, fuelZ};
    grid.at(manganesePos.x, manganesePos.y, manganesePos.z) = {
        BlockKind::Sink,
        manganeseType,
    };
    markProtected(protectedPositions, grid, manganesePos);
    fixedSinkPositions.push_back(manganesePos);

    const Pos glowstonePos{fuelX + 1, middleY, fuelZ};
    grid.at(glowstonePos.x, glowstonePos.y, glowstonePos.z) = {
        BlockKind::Sink,
        glowstoneType,
    };
    markProtected(protectedPositions, grid, glowstonePos);
    fixedSinkPositions.push_back(glowstonePos);

    if (spec.mode == DualTemplateMode::Aligned) {
        if (spec.lines[0].moderatorCount != spec.lines[1].moderatorCount) {
            return std::nullopt;
        }
        const Pos arsenicPos{
            fuelX + spec.lines[0].moderatorCount + 1,
            middleY,
            fuelZ,
        };
        grid.at(arsenicPos.x, arsenicPos.y, arsenicPos.z) = {
            BlockKind::Sink,
            arsenicType,
        };
        markProtected(protectedPositions, grid, arsenicPos);
        fixedSinkPositions.push_back(arsenicPos);
    }

    if (!placeDualSources(grid, request, fuelPositions, protectedPositions)) {
        return std::nullopt;
    }

    std::vector<Pos> endStoneCandidates =
        endStoneCandidatePositions(grid, reflectorPositions);
    for (const Pos& candidate : endStoneCandidates) {
        markProtected(protectedPositions, grid, candidate);
    }

    return BuiltDualFuelSkeleton{
        std::move(grid),
        std::move(protectedPositions),
        fuelPositions,
        reflectorPositions,
        std::move(fixedSinkPositions),
        std::move(endStoneCandidates),
        spec,
    };
}

bool canBridgeThrough(const Grid& grid, int idx,
                      const StateVector& protectedPositions,
                      const StateVector& targetMask) {
    if (targetMask.at(static_cast<size_t>(idx))) {
        return true;
    }
    if (protectedPositions.at(static_cast<size_t>(idx))) {
        return false;
    }
    const Block& block = grid.atIndex(idx);
    return block.kind == BlockKind::Empty || isSupportMutable(block);
}

std::optional<std::vector<Pos>> shortestConductorPath(
    const Grid& grid, const Pos& start, const StateVector& targetMask,
    const StateVector& protectedPositions,
    const std::atomic_bool* cancelRequested) {
    if (!grid.isInterior(start.x, start.y, start.z)) {
        return std::nullopt;
    }

    StateVector visited(static_cast<size_t>(grid.volume()), false);
    std::vector<int> previous(static_cast<size_t>(grid.volume()), -1);
    std::deque<int> queue;
    const int startIdx = grid.index(start.x, start.y, start.z);
    visited.at(static_cast<size_t>(startIdx)) = true;
    queue.push_back(startIdx);

    while (!queue.empty()) {
        throwIfCancelled(cancelRequested);
        const int idx = queue.front();
        queue.pop_front();
        if (targetMask.at(static_cast<size_t>(idx))) {
            std::vector<Pos> path;
            for (int current = idx; current >= 0;
                 current = previous.at(static_cast<size_t>(current))) {
                path.push_back(indexToPos(grid, current));
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        const Pos pos = indexToPos(grid, idx);
        grid.forEachNeighbor6(pos, [&](const Pos& neighbor) {
            if (!grid.isInterior(neighbor.x, neighbor.y, neighbor.z)) {
                return;
            }
            const int neighborIdx =
                grid.index(neighbor.x, neighbor.y, neighbor.z);
            if (visited.at(static_cast<size_t>(neighborIdx)) ||
                !canBridgeThrough(grid, neighborIdx, protectedPositions,
                                  targetMask)) {
                return;
            }
            visited.at(static_cast<size_t>(neighborIdx)) = true;
            previous.at(static_cast<size_t>(neighborIdx)) = idx;
            queue.push_back(neighborIdx);
        });
    }
    return std::nullopt;
}

bool tryConnectSinkToHeatingCluster(
    Grid& grid, FuelSimulation& sim, const Pos& sinkPos,
    StateVector& protectedPositions,
    const std::atomic_bool* cancelRequested) {
    const int sinkIdx = grid.index(sinkPos.x, sinkPos.y, sinkPos.z);
    if (grid.atIndex(sinkIdx).kind != BlockKind::Sink ||
        !sim.validSinks.at(static_cast<size_t>(sinkIdx))) {
        return false;
    }
    if (sim.heatingClusterBlocks.at(static_cast<size_t>(sinkIdx))) {
        return true;
    }
    if (std::none_of(sim.heatingClusterBlocks.begin(),
                     sim.heatingClusterBlocks.end(),
                     [](auto value) { return value != 0; })) {
        return false;
    }

    const StateVector targetMask = sim.heatingClusterBlocks;
    const std::optional<std::vector<Pos>> path =
        shortestConductorPath(grid, sinkPos, targetMask,
                              protectedPositions, cancelRequested);
    if (!path.has_value()) {
        return false;
    }

    Grid trial = grid;
    StateVector trialProtected = protectedPositions;
    for (const Pos& pathPos : *path) {
        const int idx = trial.index(pathPos.x, pathPos.y, pathPos.z);
        if (targetMask.at(static_cast<size_t>(idx)) ||
            trialProtected.at(static_cast<size_t>(idx))) {
            continue;
        }
        trial.atIndex(idx) = {BlockKind::Conductor, -1};
        trialProtected.at(static_cast<size_t>(idx)) = true;
    }

    FuelSimulation trialSim = simulateMixedFuel(trial);
    if (!isPreCompactRunnable(trialSim) ||
        !hasSafeFuelFlux(trial, trialSim) ||
        !trialSim.validSinks.at(static_cast<size_t>(sinkIdx)) ||
        !trialSim.heatingClusterBlocks.at(static_cast<size_t>(sinkIdx))) {
        return false;
    }

    grid = std::move(trial);
    protectedPositions = std::move(trialProtected);
    sim = std::move(trialSim);
    return true;
}

bool connectFixedSinks(BuiltDualFuelSkeleton& skeleton,
                       const std::atomic_bool* cancelRequested) {
    FuelSimulation sim = simulateMixedFuel(skeleton.grid);
    if (!isPreCompactRunnable(sim) ||
        !hasSafeFuelFlux(skeleton.grid, sim)) {
        return false;
    }
    for (const Pos& sinkPos : skeleton.fixedSinkPositions) {
        if (!tryConnectSinkToHeatingCluster(
                skeleton.grid, sim, sinkPos,
                skeleton.protectedPositions, cancelRequested)) {
            return false;
        }
    }
    return sim.disconnectedFunctionalBlocks == 0;
}

bool protectedLayoutPreserved(const Grid& candidate, const Grid& baseline,
                              const StateVector& protectedPositions) {
    if (candidate.volume() != baseline.volume() ||
        protectedPositions.size() !=
            static_cast<size_t>(candidate.volume())) {
        return false;
    }
    for (int idx = 0; idx < candidate.volume(); ++idx) {
        if (protectedPositions.at(static_cast<size_t>(idx)) &&
            !sameBlock(candidate.atIndex(idx), baseline.atIndex(idx))) {
            return false;
        }
    }
    return true;
}

bool validFixedSink(const Grid& grid, const FuelSimulation& sim,
                    const Pos& pos, int sinkType) {
    if (!grid.isInterior(pos.x, pos.y, pos.z)) {
        return false;
    }
    const int idx = grid.index(pos.x, pos.y, pos.z);
    const Block& block = grid.atIndex(idx);
    return block.kind == BlockKind::Sink && block.type == sinkType &&
           sim.validSinks.at(static_cast<size_t>(idx)) &&
           sim.heatingClusterBlocks.at(static_cast<size_t>(idx));
}

bool hasRequiredDualTemplate(const Grid& grid, const FuelSimulation& sim,
                             const BuildRequest& request,
                             const DualTemplateSpec& spec) {
    const std::vector<Pos> fuelsInGrid = orderedFuelPositions(grid);
    if (fuelsInGrid.size() != 2) {
        return false;
    }
    const Pos lowerFuel = fuelsInGrid[0];
    const Pos upperFuel = fuelsInGrid[1];
    if (lowerFuel.x != upperFuel.x ||
        lowerFuel.z != upperFuel.z ||
        upperFuel.y != lowerFuel.y + 2 ||
        grid.at(lowerFuel.x, lowerFuel.y, lowerFuel.z).type !=
            request.fuelIndices[0] ||
        grid.at(upperFuel.x, upperFuel.y, upperFuel.z).type !=
            request.fuelIndices[1]) {
        return false;
    }

    const int middleY = lowerFuel.y + 1;
    if (!validFixedSink(grid, sim,
                        {lowerFuel.x, middleY, lowerFuel.z},
                        manganeseSinkType()) ||
        !validFixedSink(grid, sim,
                        {lowerFuel.x + 1, middleY, lowerFuel.z},
                        glowstoneSinkType())) {
        return false;
    }

    for (int slot = 0; slot < 2; ++slot) {
        const Pos fuelPos = fuelsInGrid.at(static_cast<size_t>(slot));
        const FuelLineSpec& line = spec.lines.at(static_cast<size_t>(slot));
        for (int distance = 1; distance <= line.moderatorCount; ++distance) {
            const Pos moderatorPos{
                fuelPos.x + distance,
                fuelPos.y,
                fuelPos.z,
            };
            const int idx =
                grid.index(moderatorPos.x, moderatorPos.y, moderatorPos.z);
            const Block& block = grid.atIndex(idx);
            if (block.kind != BlockKind::Moderator ||
                block.type != line.moderatorType ||
                (distance == 1 &&
                 !sim.activeModerators.at(static_cast<size_t>(idx)))) {
                return false;
            }
        }
        const Pos reflectorPos{
            fuelPos.x + line.moderatorCount + 1,
            fuelPos.y,
            fuelPos.z,
        };
        const int reflectorIdx =
            grid.index(reflectorPos.x, reflectorPos.y, reflectorPos.z);
        const Block& reflector = grid.atIndex(reflectorIdx);
        if (reflector.kind != BlockKind::Reflector ||
            reflector.type != line.reflectorType ||
            !sim.activeReflectors.at(static_cast<size_t>(reflectorIdx))) {
            return false;
        }
    }

    if (spec.mode == DualTemplateMode::Aligned) {
        const Pos arsenicPos{
            lowerFuel.x + spec.lines[0].moderatorCount + 1,
            middleY,
            lowerFuel.z,
        };
        if (!validFixedSink(grid, sim, arsenicPos, arsenicSinkType())) {
            return false;
        }
    }
    return hasRequiredSources(grid, request);
}

int functionalEndStoneCount(const Grid& grid, const FuelSimulation& sim) {
    const int endStoneType = endStoneSinkType();
    int count = 0;
    for (const Pos& pos : grid.interiorPositions()) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        const Block& block = grid.atIndex(idx);
        if (block.kind == BlockKind::Sink &&
            block.type == endStoneType &&
            sim.validSinks.at(static_cast<size_t>(idx)) &&
            sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            ++count;
        }
    }
    return count;
}

bool hasFunctionalManaDustCorners(const Grid& grid,
                                  const FuelSimulation& sim) {
    const int manaType = manaDustSinkType();
    if (manaType < 0 || grid.internalA() < 2 ||
        grid.internalB() < 2 || grid.internalC() < 2) {
        return false;
    }
    for (const Pos& corner : interiorCornerPositions(grid)) {
        const int idx = grid.index(corner.x, corner.y, corner.z);
        const Block& block = grid.atIndex(idx);
        if (block.kind != BlockKind::Sink ||
            block.type != manaType ||
            !sim.validSinks.at(static_cast<size_t>(idx)) ||
            !sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            return false;
        }
    }
    return true;
}

std::optional<OptimizationResult> acceptedDualResult(
    Grid grid, const BuildRequest& request, const DualTemplateSpec& spec,
    int minimumEndStoneCount = 0, bool requireManaDust = false) {
    FuelSimulation sim = simulateMixedFuel(grid);
    if (!isSearchAccepted(grid, sim)) {
        return std::nullopt;
    }

    Grid compacted = compactEmptyInteriorPlanes(std::move(grid));
    if (!hasNoEmptyInteriorPlane(compacted)) {
        return std::nullopt;
    }

    FuelSimulation compactedSim = simulateMixedFuel(compacted);
    if (!isSearchAccepted(compacted, compactedSim) ||
        !evaluateHeatingClusterWallConnections(
             compacted, compactedSim).allConnected() ||
        !hasRequiredDualTemplate(
             compacted, compactedSim, request, spec) ||
        functionalEndStoneCount(compacted, compactedSim) <
            minimumEndStoneCount ||
        (requireManaDust &&
         !hasFunctionalManaDustCorners(compacted, compactedSim))) {
        return std::nullopt;
    }

    try {
        OptimizationResult result =
            resultFromSimulation(
                std::move(compacted), request, compactedSim);
        FuelSimulation finalSim = simulateMixedFuel(result.grid);
        if (!isFinalReactorValid(result.grid, request, finalSim) ||
            !hasRequiredDualTemplate(result.grid, finalSim, request, spec) ||
            functionalEndStoneCount(result.grid, finalSim) <
                minimumEndStoneCount ||
            (requireManaDust &&
             !hasFunctionalManaDustCorners(result.grid, finalSim))) {
            return std::nullopt;
        }
        return result;
    } catch (const std::runtime_error&) {
        return std::nullopt;
    }
}

std::optional<PaddedDualGrid> compactAndPadForManaDust(
    const Grid& grid, const BuildRequest& request,
    const DualTemplateSpec& spec) {
    Grid compacted = compactEmptyInteriorPlanes(grid);
    FuelSimulation compactedSim = simulateMixedFuel(compacted);
    if (!isPreCompactRunnable(compactedSim) ||
        !hasSafeFuelFlux(compacted, compactedSim) ||
        !hasRequiredDualTemplate(compacted, compactedSim, request, spec) ||
        compacted.internalA() + 2 > kMaxSize ||
        compacted.internalB() + 2 > kMaxSize ||
        compacted.internalC() + 2 > kMaxSize) {
        return std::nullopt;
    }

    Grid padded = makeShell(compacted.internalA() + 2,
                            compacted.internalB() + 2,
                            compacted.internalC() + 2);
    for (const Pos& pos : compacted.interiorPositions()) {
        const Block& block = compacted.at(pos.x, pos.y, pos.z);
        if (block.kind != BlockKind::Empty) {
            padded.at(pos.x + 1, pos.y + 1, pos.z + 1) = block;
        }
    }

    StateVector protectedPositions(static_cast<size_t>(padded.volume()), false);
    for (const Pos& pos : padded.interiorPositions()) {
        if (padded.at(pos.x, pos.y, pos.z).kind != BlockKind::Empty) {
            markProtected(protectedPositions, padded, pos);
        }
    }

    const std::vector<Pos> fuelPositions = orderedFuelPositions(padded);
    if (fuelPositions.size() != 2) {
        return std::nullopt;
    }
    for (int slot = 0; slot < 2; ++slot) {
        const Fuel& fuel =
            fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(slot))));
        if (fuel.selfPriming) {
            continue;
        }
        const Pos fuelPos = fuelPositions.at(static_cast<size_t>(slot));
        for (int x = 1; x < fuelPos.x; ++x) {
            const Pos pathPos{x, fuelPos.y, fuelPos.z};
            if (padded.at(pathPos.x, pathPos.y, pathPos.z).kind !=
                BlockKind::Empty) {
                return std::nullopt;
            }
        }
        const Pos sourcePos{0, fuelPos.y, fuelPos.z};
        padded.at(sourcePos.x, sourcePos.y, sourcePos.z) = {
            BlockKind::Source,
            -1,
        };
        markProtected(protectedPositions, padded, sourcePos);
        if (sourcePrimingTargetIndex(padded, sourcePos) !=
            padded.index(fuelPos.x, fuelPos.y, fuelPos.z)) {
            return std::nullopt;
        }
    }

    FuelSimulation paddedSim = simulateMixedFuel(padded);
    if (!isPreCompactRunnable(paddedSim) ||
        !hasSafeFuelFlux(padded, paddedSim) ||
        !hasRequiredDualTemplate(padded, paddedSim, request, spec)) {
        return std::nullopt;
    }
    return PaddedDualGrid{
        std::move(padded),
        std::move(protectedPositions),
    };
}

#ifndef NDEBUG
const char* dualModeName(DualTemplateMode mode) {
    return mode == DualTemplateMode::Aligned ? "aligned" : "unequal";
}

void logDualCandidate(const char* reason, const DualTemplateSpec& spec,
                      const Grid* grid = nullptr,
                      const FuelSimulation* sim = nullptr) {
    std::ostringstream os;
    os << "reason=" << reason
       << " mode=" << dualModeName(spec.mode)
       << " line0=" << spec.lines[0].moderatorCount
       << " line1=" << spec.lines[1].moderatorCount
       << " moderator0=" << spec.lines[0].moderatorType
       << " moderator1=" << spec.lines[1].moderatorType
       << " reflector0=" << spec.lines[0].reflectorType
       << " reflector1=" << spec.lines[1].reflectorType;
    if (grid != nullptr) {
        os << " grid=" << gridInteriorLabel(*grid);
    }
    if (sim != nullptr) {
        os << " compatible=" << (sim->compatible ? 1 : 0)
           << " safeFlux=" << (grid != nullptr && hasSafeFuelFlux(*grid, *sim) ? 1 : 0)
           << " rawHeating=" << sim->rawHeating
           << " cooling=" << sim->cooling
           << " deficit=" << (sim->rawHeating - sim->cooling)
           << " disconnected=" << sim->disconnectedFunctionalBlocks;
    }
    const std::string detail = os.str();
    NCFR_PERF_CHECKPOINT("dual.candidate", detail.c_str());
}

void logDualFallback(const char* reason, const DualTemplateSpec& spec,
                     const FuelSimulation& sim, int endStoneCandidates,
                     int endStonePlaced, long long threshold,
                     bool manaDustUsed) {
    std::ostringstream os;
    os << "reason=" << reason
       << " mode=" << dualModeName(spec.mode)
       << " line0=" << spec.lines[0].moderatorCount
       << " line1=" << spec.lines[1].moderatorCount
       << " rawHeating=" << sim.rawHeating
       << " cooling=" << sim.cooling
       << " deficit=" << (sim.rawHeating - sim.cooling)
       << " endStoneCandidates=" << endStoneCandidates
       << " endStonePlaced=" << endStonePlaced
       << " threshold=" << threshold
       << " manaDust=" << (manaDustUsed ? 1 : 0);
    const std::string detail = os.str();
    NCFR_PERF_CHECKPOINT("dual.fallback", detail.c_str());
}
#endif

std::optional<OptimizationResult> tryDualCoolingFallback(
    Grid grid, FuelSimulation sim, StateVector protectedPositions,
    const BuildRequest& request, const DualTemplateSpec& spec,
    const std::vector<Pos>& endStoneCandidates,
    const std::atomic_bool* cancelRequested) {
    if (classifyFinalizationFailure(grid, sim, request) !=
            FinalizeFailureKind::CoolingDeficit ||
        !isPreCompactRunnable(sim) ||
        !hasSafeFuelFlux(grid, sim) ||
        sim.disconnectedFunctionalBlocks != 0) {
        return std::nullopt;
    }

    const long long initialDeficit = sim.rawHeating - sim.cooling;
    const int endStoneType = endStoneSinkType();
    if (initialDeficit <= 0 || endStoneType < 0) {
        return std::nullopt;
    }

    int placedEndStone = 0;
    for (const Pos& candidate : endStoneCandidates) {
        throwIfCancelled(cancelRequested);
        Grid trial = grid;
        StateVector trialProtected = protectedPositions;
        trial.at(candidate.x, candidate.y, candidate.z) = {
            BlockKind::Sink,
            endStoneType,
        };
        markProtected(trialProtected, trial, candidate);
        FuelSimulation trialSim = simulateMixedFuel(trial);
        if (!isPreCompactRunnable(trialSim) ||
            !hasSafeFuelFlux(trial, trialSim) ||
            !trialSim.validSinks.at(static_cast<size_t>(
                trial.index(candidate.x, candidate.y, candidate.z))) ||
            !tryConnectSinkToHeatingCluster(
                trial, trialSim, candidate, trialProtected,
                cancelRequested)) {
            continue;
        }
        grid = std::move(trial);
        protectedPositions = std::move(trialProtected);
        sim = std::move(trialSim);
        ++placedEndStone;
    }

    const long long threshold =
        static_cast<long long>(placedEndStone) * 65 +
        kManaDustFallbackCapacity;
#ifndef NDEBUG
    logDualFallback("endStoneComplete", spec, sim,
                    static_cast<int>(endStoneCandidates.size()),
                    placedEndStone, threshold, false);
#endif
    if (initialDeficit > threshold) {
        return std::nullopt;
    }

    if (isSearchAccepted(grid, sim)) {
        return acceptedDualResult(
            std::move(grid), request, spec, placedEndStone, false);
    }

    const long long remainingDeficit = sim.rawHeating - sim.cooling;
    if (remainingDeficit <= 0 ||
        remainingDeficit > kManaDustFallbackCapacity) {
        return std::nullopt;
    }

    std::optional<PaddedDualGrid> padded =
        compactAndPadForManaDust(grid, request, spec);
    if (!padded.has_value()) {
        return std::nullopt;
    }

    Grid manaGrid = std::move(padded->grid);
    StateVector manaProtected = std::move(padded->protectedPositions);
    FuelSimulation manaSim = simulateMixedFuel(manaGrid);
    const int manaType = manaDustSinkType();
    if (manaType < 0) {
        return std::nullopt;
    }

    for (const Pos& corner : interiorCornerPositions(manaGrid)) {
        throwIfCancelled(cancelRequested);
        if (manaGrid.at(corner.x, corner.y, corner.z).kind !=
            BlockKind::Empty) {
            return std::nullopt;
        }
        manaGrid.at(corner.x, corner.y, corner.z) = {
            BlockKind::Sink,
            manaType,
        };
        markProtected(manaProtected, manaGrid, corner);
        manaSim = simulateMixedFuel(manaGrid);
        if (!manaSim.validSinks.at(static_cast<size_t>(
                manaGrid.index(corner.x, corner.y, corner.z))) ||
            !tryConnectSinkToHeatingCluster(
                manaGrid, manaSim, corner, manaProtected,
                cancelRequested)) {
            return std::nullopt;
        }
    }

#ifndef NDEBUG
    logDualFallback("manaDustComplete", spec, manaSim,
                    static_cast<int>(endStoneCandidates.size()),
                    placedEndStone, threshold, true);
#endif
    if (!isSearchAccepted(manaGrid, manaSim) ||
        !hasFunctionalManaDustCorners(manaGrid, manaSim)) {
        return std::nullopt;
    }
    return acceptedDualResult(
        std::move(manaGrid), request, spec, placedEndStone, true);
}

std::optional<OptimizationResult> tryFinalizeDualCandidate(
    BuiltDualFuelSkeleton skeleton, const BuildRequest& request,
    const std::atomic_bool* cancelRequested) {
    NCFR_PERF_COUNT(finalizeCandidateCalls);
    NCFR_PERF_SCOPE(finalizeCandidateNs);
    if (!connectFixedSinks(skeleton, cancelRequested)) {
        return std::nullopt;
    }

    FuelSimulation sim = simulateMixedFuel(skeleton.grid);
#ifndef NDEBUG
    logDualCandidate("connectedSkeleton", skeleton.spec,
                     &skeleton.grid, &sim);
#endif
    if (!isPreCompactRunnable(sim) ||
        !hasSafeFuelFlux(skeleton.grid, sim) ||
        sim.disconnectedFunctionalBlocks != 0) {
        return std::nullopt;
    }

    if (isSearchAccepted(skeleton.grid, sim)) {
        if (std::optional<OptimizationResult> result =
                acceptedDualResult(
                    skeleton.grid, request, skeleton.spec)) {
            return result;
        }
    }

    const SupportBlockOptions supportOptions{
        request.selectedModeratorTypeIndices,
        request.selectedReflectorTypeIndices,
    };
    fillSupportBlocks(
        skeleton.grid, &supportOptions,
        &skeleton.protectedPositions);
    pruneInactiveSupport(
        skeleton.grid, &skeleton.protectedPositions);
    sim = simulateMixedFuel(skeleton.grid);
    if (isSearchAccepted(skeleton.grid, sim)) {
        if (std::optional<OptimizationResult> result =
                acceptedDualResult(
                    skeleton.grid, request, skeleton.spec)) {
            return result;
        }
    }

    skeleton.grid = improveSupportBlocks(
        std::move(skeleton.grid), cancelRequested,
        kDefaultImproveOptions, &supportOptions,
        &skeleton.protectedPositions, true);
    pruneInactiveSupport(
        skeleton.grid, &skeleton.protectedPositions);
    sim = simulateMixedFuel(skeleton.grid);
    if (isSearchAccepted(skeleton.grid, sim)) {
        if (std::optional<OptimizationResult> result =
                acceptedDualResult(
                    skeleton.grid, request, skeleton.spec)) {
            return result;
        }
    }

    if (classifyFinalizationFailure(
            skeleton.grid, sim, request) ==
        FinalizeFailureKind::CoolingDeficit) {
        const Grid protectedBaseline = skeleton.grid;
        const std::vector<SourcePrimingTarget> expectedSourceTargets =
            sourcePrimingTargets(skeleton.grid);
        skeleton.grid = expandCoolingWithPreserver(
            std::move(skeleton.grid),
            [&protectedBaseline,
             &protectedPositions = skeleton.protectedPositions,
             &expectedSourceTargets](
                Grid& candidate) {
                return protectedLayoutPreserved(
                           candidate, protectedBaseline,
                           protectedPositions) &&
                       matchesSourcePrimingTargets(
                           candidate, expectedSourceTargets);
            },
            cancelRequested, kCoolingExpansionOptions);
        pruneInactiveSupport(
            skeleton.grid, &skeleton.protectedPositions);
        sim = simulateMixedFuel(skeleton.grid);
    }

    if (isSearchAccepted(skeleton.grid, sim)) {
        if (std::optional<OptimizationResult> result =
                acceptedDualResult(
                    skeleton.grid, request, skeleton.spec)) {
            return result;
        }
    }

    return tryDualCoolingFallback(
        std::move(skeleton.grid), std::move(sim),
        std::move(skeleton.protectedPositions),
        request, skeleton.spec, skeleton.endStoneCandidates,
        cancelRequested);
}

std::vector<FuelLineSpec> dualFuelLineOptions(
    const Fuel& fuel, const BuildRequest& request) {
    std::vector<int> sourceDirections;
    if (!fuel.selfPriming) {
        sourceDirections.push_back(kSourceDirection);
    }
    std::vector<FuelLineSpec> options =
        singleFuelLineOptions(
            fuel, request, sourceDirections,
            kFuelLineDirection);
    options.erase(
        std::remove_if(
            options.begin(), options.end(),
            [&](const FuelLineSpec& line) {
                return line.estimatedFlux + kFluxEpsilon <
                       fuel.criticality;
            }),
        options.end());
    return options;
}

std::optional<OptimizationResult> searchDualTemplateMode(
    const BuildRequest& request,
    const std::array<std::vector<FuelLineSpec>, 2>& options,
    DualTemplateMode mode,
    const std::atomic_bool* cancelRequested) {
    for (const FuelLineSpec& first : options[0]) {
        for (const FuelLineSpec& second : options[1]) {
            throwIfCancelled(cancelRequested);
            const bool equalLength =
                first.moderatorCount == second.moderatorCount;
            if ((mode == DualTemplateMode::Aligned && !equalLength) ||
                (mode == DualTemplateMode::Unequal && equalLength)) {
                continue;
            }

            const DualTemplateSpec spec{{first, second}, mode};
#ifndef NDEBUG
            logDualCandidate("start", spec);
#endif
            std::optional<BuiltDualFuelSkeleton> skeleton =
                buildDualFuelSkeleton(request, spec);
            if (!skeleton.has_value()) {
#ifndef NDEBUG
                logDualCandidate("skeletonRejected", spec);
#endif
                continue;
            }

            const FuelRelationPrefilterResult relation =
                prefilterFuelRelations(skeleton->grid, request);
            if (!relation.accepted) {
#ifndef NDEBUG
                logDualCandidate("fuelRelationRejected", spec,
                                 &skeleton->grid);
#endif
                continue;
            }

            NCFR_PERF_COUNT(candidateCount);
            NCFR_PERF_COUNT(candidateEvaluations);
            NCFR_PERF_SCOPE(candidateEvaluationNs);
            if (std::optional<OptimizationResult> result =
                    tryFinalizeDualCandidate(
                        std::move(*skeleton), request,
                        cancelRequested)) {
                NCFR_PERF_COUNT(bestUpdates);
                return result;
            }
#ifndef NDEBUG
            logDualCandidate("finalizeRejected", spec);
#endif
        }
    }
    return std::nullopt;
}

} // namespace

OptimizationResult optimizeDualFuelTemplateLayoutDeprecated(
    const BuildRequest& request,
    const std::atomic_bool* cancelRequested) {
    if (request.fuelIndices.size() != 2) {
        throw std::invalid_argument(
            "双燃料策略需要 2 个燃料单元。");
    }

    const std::array<std::vector<FuelLineSpec>, 2> options{{
        dualFuelLineOptions(
            fuels().at(static_cast<size_t>(request.fuelIndices[0])),
            request),
        dualFuelLineOptions(
            fuels().at(static_cast<size_t>(request.fuelIndices[1])),
            request),
    }};
    if (options[0].empty() || options[1].empty()) {
        throw std::runtime_error(
            "双燃料模板无法为输入燃料找到安全的反射器线路。");
    }

    if (std::optional<OptimizationResult> result =
            searchDualTemplateMode(
                request, options, DualTemplateMode::Aligned,
                cancelRequested)) {
        return std::move(*result);
    }
    if (std::optional<OptimizationResult> result =
            searchDualTemplateMode(
                request, options, DualTemplateMode::Unequal,
                cancelRequested)) {
        return std::move(*result);
    }

    throw std::runtime_error(
        "无满足双燃料输入要求的模板生成方案。");
}

} // namespace ncfr::optimizer_detail
