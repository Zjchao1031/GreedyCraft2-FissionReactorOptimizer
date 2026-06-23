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

void throwIfCancelled(const std::atomic_bool* cancelRequested) {
    if (cancelRequested != nullptr && cancelRequested->load()) {
        throw OptimizationCanceled();
    }
}

int dimensionVolume(const Dimension& dim) {
    return dim.a * dim.b * dim.c;
}

int dimensionSpread(const Dimension& dim) {
    return std::max({dim.a, dim.b, dim.c}) - std::min({dim.a, dim.b, dim.c});
}

int dimensionSurface(const Dimension& dim) {
    return dim.a * dim.b + dim.a * dim.c + dim.b * dim.c;
}

#ifndef NDEBUG
std::string dimensionLabel(const Dimension& dim) {
    std::ostringstream os;
    os << dim.a << "x" << dim.b << "x" << dim.c;
    return os.str();
}

std::string gridInteriorLabel(const Grid& grid) {
    std::ostringstream os;
    os << grid.internalA() << "x" << grid.internalB() << "x" << grid.internalC();
    return os.str();
}

const char* directionLabel(int direction) {
    switch (direction) {
    case 0: return "+x";
    case 1: return "-x";
    case 2: return "+y";
    case 3: return "-y";
    case 4: return "+z";
    case 5: return "-z";
    default: return "none";
    }
}

std::string directionListLabel(const std::vector<int>& directions) {
    if (directions.empty()) {
        return "none";
    }
    std::ostringstream os;
    for (size_t i = 0; i < directions.size(); ++i) {
        if (i > 0) {
            os << ",";
        }
        os << directionLabel(directions.at(i));
    }
    return os.str();
}

std::string directionalCandidateDetail(const char* reason, const Dimension& dim,
                                       const std::vector<int>& sourceDirections,
                                       const std::vector<int>& reflectorDirections) {
    std::ostringstream os;
    os << "reason=" << reason
       << " dim=" << dimensionLabel(dim)
       << " sourceDirs=" << directionListLabel(sourceDirections)
       << " reflectorDirs=" << directionListLabel(reflectorDirections);
    return os.str();
}

std::string directionalGridDetail(const char* reason, const Grid& grid, const FuelSimulation* sim,
                                  const BuildRequest& request, const std::vector<int>& sourceDirections,
                                  const std::vector<int>& reflectorDirections) {
    (void)request;
    std::ostringstream os;
    os << "reason=" << reason
       << " grid=" << gridInteriorLabel(grid)
       << " sourceDirs=" << directionListLabel(sourceDirections)
       << " reflectorDirs=" << directionListLabel(reflectorDirections);
    if (sim != nullptr) {
        os << " compatible=" << (sim->compatible ? 1 : 0)
           << " fuelCells=" << sim->fuelCells
           << " runningCells=" << sim->runningCells
           << " minMargin=" << sim->minClusterMargin
           << " disconnected=" << sim->disconnectedFunctionalBlocks
           << " functionalIrradiators="
           << std::count(sim->functionalIrradiators.begin(), sim->functionalIrradiators.end(), true)
           << " rawHeating=" << sim->rawHeating
           << " cooling=" << sim->cooling;
    }
    return os.str();
}

std::string fuelRelationDetail(const char* reason, const FuelRelationPrefilterResult& result,
                               const BuildRequest& request) {
    (void)request;
    std::ostringstream os;
    os << "reason=" << reason
       << " relationReason=" << fuelRelationRejectReasonName(result.reason)
       << " fuelCells=" << result.fuelCells
       << " runningCells=" << result.runningCells
       << " functionalIrradiators=" << result.functionalIrradiators
       << " weakestFuelMargin=" << result.weakestFuelMargin;
    return os.str();
}

void logFinalizeCheckpoint(const char* checkpointName, const std::string& detail, int paddingPlanes,
                           const ImproveOptions& improveOptions) {
    std::ostringstream os;
    os << detail << " padding=" << paddingPlanes << " frontierRadius=" << improveOptions.frontierRadius;
    NCFR_PERF_CHECKPOINT(checkpointName, os.str().c_str());
}

void logCoolingExpansionCheckpoint(const char* reason, const Grid& grid, const FuelSimulation& sim, int pass,
                                   const Pos* pos, const SinkType* sink, long long oldMargin,
                                   const Pos* bridgePos, const SinkType* bridgeSink) {
    std::ostringstream os;
    os << "reason=" << reason
       << " grid=" << gridInteriorLabel(grid)
       << " margin=" << sim.minClusterMargin
       << " rawHeating=" << sim.rawHeating
       << " cooling=" << sim.cooling;
    if (pass >= 0) {
        os << " pass=" << pass;
    }
    if (pos != nullptr) {
        os << " pos=" << pos->x << "," << pos->y << "," << pos->z;
    }
    if (sink != nullptr) {
        os << " sink=" << sink->sourceName << " sinkCooling=" << sink->cooling;
    }
    if (bridgePos != nullptr) {
        os << " bridgePos=" << bridgePos->x << "," << bridgePos->y << "," << bridgePos->z;
    }
    if (bridgeSink != nullptr) {
        os << " bridgeSink=" << bridgeSink->sourceName << " bridgeSinkCooling=" << bridgeSink->cooling;
    }
    if (oldMargin != 0) {
        os << " oldMargin=" << oldMargin;
    }
    NCFR_PERF_CHECKPOINT("coolingExpand", os.str().c_str());
}


void recordCoolingExpansionRejection(CoolingExpansionPassStats& stats, const char* reason,
                                     const FuelSimulation* sim) {
    if (reason == nullptr) {
        reason = "unknown";
    }
    if (std::string(reason) == "restoreLineFailed") {
        ++stats.restoreLineFailed;
    } else if (std::string(reason) == "invalidNewSink") {
        ++stats.invalidNewSink;
    } else if (std::string(reason) == "invalidBridgeTarget") {
        ++stats.invalidBridgeTarget;
    } else if (std::string(reason) == "invalidBridgeSink") {
        ++stats.invalidBridgeSink;
    } else if (std::string(reason) == "notRunnable") {
        ++stats.notRunnable;
    } else if (std::string(reason) == "disconnected") {
        ++stats.disconnected;
    } else if (std::string(reason) == "noMarginGain") {
        ++stats.noMarginGain;
    } else if (std::string(reason) == "notBest") {
        ++stats.notBest;
    }
    if (sim != nullptr && sim->minClusterMargin > stats.bestRejectedMargin) {
        stats.bestRejectedMargin = sim->minClusterMargin;
        stats.bestRejectedCooling = sim->cooling;
        stats.bestRejectedReason = reason;
    }
}

void logCoolingExpansionStats(const char* reason, const Grid& grid, const FuelSimulation& sim, int pass,
                              const CoolingExpansionPassStats& stats) {
    std::ostringstream os;
    os << "reason=" << reason
       << " grid=" << gridInteriorLabel(grid)
       << " pass=" << pass
       << " margin=" << sim.minClusterMargin
       << " positions=" << stats.positions
       << " directPositions=" << stats.directPositions
       << " bridgeTargetPositions=" << stats.bridgeTargetPositions
       << " clusterConnectedPositions=" << stats.clusterConnectedPositions
       << " sinkTypes=" << stats.sinkTypes
       << " ruleChecks=" << stats.ruleChecks
       << " ruleValidSinks=" << stats.ruleValidSinks
       << " bridgeRuleChecks=" << stats.bridgeRuleChecks
       << " bridgeRuleValidSinks=" << stats.bridgeRuleValidSinks
       << " singleCandidates=" << stats.singleCandidates
       << " bridgeTargetCandidates=" << stats.bridgeTargetCandidates
       << " bridgeCandidates=" << stats.bridgeCandidates
       << " selectedCandidates=" << stats.selectedCandidates
       << " trials=" << stats.trials
       << " bridgeTrials=" << stats.bridgeTrials
       << " restoreLineFailed=" << stats.restoreLineFailed
       << " invalidNewSink=" << stats.invalidNewSink
       << " invalidBridgeTarget=" << stats.invalidBridgeTarget
       << " invalidBridgeSink=" << stats.invalidBridgeSink
       << " notRunnable=" << stats.notRunnable
       << " disconnected=" << stats.disconnected
       << " noMarginGain=" << stats.noMarginGain
       << " notBest=" << stats.notBest
       << " newBest=" << stats.newBest
       << " bridgeNewBest=" << stats.bridgeNewBest;
    if (stats.bestRejectedMargin != std::numeric_limits<long long>::min()) {
        os << " bestRejectedReason=" << stats.bestRejectedReason
           << " bestRejectedMargin=" << stats.bestRejectedMargin
           << " bestRejectedCooling=" << stats.bestRejectedCooling;
    }
    NCFR_PERF_CHECKPOINT("coolingExpandStats", os.str().c_str());
}
#endif

void validateIndex(int index, int size, const char* label) {
    if (index < 0 || index >= size) {
        throw std::invalid_argument(std::string(label) + "索引超出范围。");
    }
}

int requiredSourceCountForFuels(const BuildRequest& request) {
    int count = 0;
    for (int fuelIndex : request.fuelIndices) {
        if (!fuels().at(static_cast<size_t>(fuelIndex)).selfPriming) {
            ++count;
        }
    }
    return count;
}

void validateRequest(const BuildRequest& request) {
    if (request.fuelIndices.empty()) {
        throw std::invalid_argument("燃料单元数量至少为 1。");
    }
    if (request.fuelIndices.size() != 1 && request.fuelIndices.size() != 2 &&
        request.fuelIndices.size() != 4 && request.fuelIndices.size() != 5) {
        throw std::invalid_argument("燃料单元数量只能为 1、2、4 或 5。");
    }
    for (int index : request.fuelIndices) {
        validateIndex(index, static_cast<int>(fuels().size()), "燃料");
    }
    if (request.fuelIndices.size() == 5) {
        if (request.selectedModeratorTypeIndices.empty()) {
            throw std::invalid_argument("辐照结构生成需要至少选择一个减速剂。");
        }
        if (request.selectedReflectorTypeIndices.empty()) {
            throw std::invalid_argument("辐照结构生成需要至少选择一个中子反射器。");
        }
        validateIndex(request.irradiatorRecipeIndex, static_cast<int>(irradiatorRecipeTypes().size()), "辐照配方");
    }
    for (int index : request.selectedModeratorTypeIndices) {
        validateIndex(index, static_cast<int>(moderatorTypes().size()), "减速剂");
    }
    for (int index : request.selectedReflectorTypeIndices) {
        validateIndex(index, static_cast<int>(reflectorTypes().size()), "中子反射器");
    }
}

std::vector<Dimension> sortedDimensions() {
    std::vector<Dimension> dims;
    dims.reserve(kMaxSize * kMaxSize * kMaxSize);
    for (int a = 1; a <= kMaxSize; ++a) {
        for (int b = 1; b <= kMaxSize; ++b) {
            for (int c = 1; c <= kMaxSize; ++c) {
                dims.push_back({a, b, c});
            }
        }
    }
    std::sort(dims.begin(), dims.end(), [](const Dimension& lhs, const Dimension& rhs) {
        const int lhsVolume = dimensionVolume(lhs);
        const int rhsVolume = dimensionVolume(rhs);
        if (lhsVolume != rhsVolume) {
            return lhsVolume < rhsVolume;
        }
        const int lhsSpread = dimensionSpread(lhs);
        const int rhsSpread = dimensionSpread(rhs);
        if (lhsSpread != rhsSpread) {
            return lhsSpread < rhsSpread;
        }
        const int lhsSurface = dimensionSurface(lhs);
        const int rhsSurface = dimensionSurface(rhs);
        if (lhsSurface != rhsSurface) {
            return lhsSurface < rhsSurface;
        }
        if (lhs.a != rhs.a) return lhs.a < rhs.a;
        if (lhs.b != rhs.b) return lhs.b < rhs.b;
        return lhs.c < rhs.c;
    });
    return dims;
}

[[maybe_unused]] std::vector<Dimension> maxFirstDimensions() {
    std::vector<Dimension> dims = sortedDimensions();
    std::sort(dims.begin(), dims.end(), [](const Dimension& lhs, const Dimension& rhs) {
        const int lhsVolume = dimensionVolume(lhs);
        const int rhsVolume = dimensionVolume(rhs);
        if (lhsVolume != rhsVolume) {
            return lhsVolume > rhsVolume;
        }
        const int lhsSpread = dimensionSpread(lhs);
        const int rhsSpread = dimensionSpread(rhs);
        if (lhsSpread != rhsSpread) {
            return lhsSpread < rhsSpread;
        }
        const int lhsSurface = dimensionSurface(lhs);
        const int rhsSurface = dimensionSurface(rhs);
        if (lhsSurface != rhsSurface) {
            return lhsSurface < rhsSurface;
        }
        if (lhs.a != rhs.a) return lhs.a < rhs.a;
        if (lhs.b != rhs.b) return lhs.b < rhs.b;
        return lhs.c < rhs.c;
    });
    return dims;
}

int adjacentCells(const Grid& grid, const Pos& pos) {
    int count = 0;
    grid.forEachNeighbor6(pos, [&](const Pos& n) {
        if (grid.at(n.x, n.y, n.z).kind == BlockKind::FuelCell) {
            ++count;
        }
    });
    return count;
}

bool isBetweenFuelCells(const Grid& grid, const Pos& pos, int axis) {
    Pos a = pos;
    Pos b = pos;
    if (axis == 0) {
        --a.x;
        ++b.x;
    } else if (axis == 1) {
        --a.y;
        ++b.y;
    } else {
        --a.z;
        ++b.z;
    }
    return grid.inBounds(a.x, a.y, a.z) && grid.inBounds(b.x, b.y, b.z) &&
           grid.at(a.x, a.y, a.z).kind == BlockKind::FuelCell &&
           grid.at(b.x, b.y, b.z).kind == BlockKind::FuelCell;
}

RuleContext optimisticRuleContext(const Grid& grid, StateVector& validSinks, StateVector& functionalCells,
                                  StateVector& activeModerators, StateVector& activeReflectors,
                                  StateVector& functionalShields, StateVector& functionalIrradiators) {
    NCFR_PERF_COUNT(optimisticRuleContextBuilds);
    const size_t volume = static_cast<size_t>(grid.volume());
    validSinks.assign(volume, true);
    functionalCells.assign(volume, false);
    activeModerators.assign(volume, false);
    activeReflectors.assign(volume, false);
    functionalShields.assign(volume, false);
    functionalIrradiators.assign(volume, false);
    for (const Pos& pos : grid.interiorPositions()) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        const Block& block = grid.atIndex(idx);
        functionalCells.at(static_cast<size_t>(idx)) = block.kind == BlockKind::FuelCell;
        activeModerators.at(static_cast<size_t>(idx)) = block.kind == BlockKind::Moderator;
        activeReflectors.at(static_cast<size_t>(idx)) = block.kind == BlockKind::Reflector;
        functionalShields.at(static_cast<size_t>(idx)) = block.kind == BlockKind::Shield;
        functionalIrradiators.at(static_cast<size_t>(idx)) = block.kind == BlockKind::Irradiator;
    }
    RuleContext context;
    context.validSinks = &validSinks;
    context.functionalCells = &functionalCells;
    context.activeModerators = &activeModerators;
    context.activeReflectors = &activeReflectors;
    context.functionalShields = &functionalShields;
    context.functionalIrradiators = &functionalIrradiators;
    return context;
}

bool optimisticSinkValidAt(Grid& grid, const Pos& pos, const RuleContext& context) {
    NCFR_PERF_COUNT(optimisticSinkChecks);
    NCFR_PERF_SCOPE(optimisticSinkNs);
    return isSinkValidAt(grid, pos, context);
}

int manaDustSinkType() {
    static const int type = [] {
        for (const SinkType& sink : sinkTypes()) {
            if (sink.sourceName == "mana_dust") {
                return sink.index;
            }
        }
        return -1;
    }();
    return type;
}

bool isManaDustSink(const Block& block) {
    const int type = manaDustSinkType();
    return type >= 0 && block.kind == BlockKind::Sink && block.type == type;
}

bool isInteriorCorner(const Grid& grid, const Pos& pos) {
    return grid.isInterior(pos.x, pos.y, pos.z) &&
           (pos.x == 1 || pos.x == grid.internalA()) &&
           (pos.y == 1 || pos.y == grid.internalB()) &&
           (pos.z == 1 || pos.z == grid.internalC());
}

bool cornerSinkConnectsToInteriorCluster(const Grid& grid, const Pos& corner) {
    StateVector visited(static_cast<size_t>(grid.volume()), false);
    std::deque<int> queue;
    const int start = grid.index(corner.x, corner.y, corner.z);
    visited.at(static_cast<size_t>(start)) = true;
    queue.push_back(start);

    while (!queue.empty()) {
        const int idx = queue.front();
        queue.pop_front();
        const int x = idx % grid.width();
        const int yz = idx / grid.width();
        const int y = yz % grid.height();
        const int z = yz / grid.height();
        const Pos pos{x, y, z};
        if (idx != start && !isInteriorCorner(grid, pos)) {
            return true;
        }

        grid.forEachNeighbor6(pos, [&](const Pos& n) {
            if (!grid.isInterior(n.x, n.y, n.z)) {
                return;
            }
            const int nIdx = grid.index(n.x, n.y, n.z);
            if (visited.at(static_cast<size_t>(nIdx)) ||
                !isFunctionalInterior(grid.atIndex(nIdx).kind)) {
                return;
            }
            visited.at(static_cast<size_t>(nIdx)) = true;
            queue.push_back(nIdx);
        });
    }
    return false;
}

void removeUnclusteredCornerManaDustSinks(Grid& grid) {
    const int xs[2] = {1, grid.internalA()};
    const int ys[2] = {1, grid.internalB()};
    const int zs[2] = {1, grid.internalC()};
    for (int x : xs) {
        for (int y : ys) {
            for (int z : zs) {
                Block& block = grid.at(x, y, z);
                if (isManaDustSink(block) && !cornerSinkConnectsToInteriorCluster(grid, {x, y, z})) {
                    block = {BlockKind::Empty, -1};
                }
            }
        }
    }
}

const SupportBlockOptions& defaultSupportBlockOptions() {
    static const SupportBlockOptions options{{2}, {0, 1}};
    return options;
}

const SupportBlockOptions& effectiveSupportBlockOptions(const SupportBlockOptions* options) {
    return options == nullptr ? defaultSupportBlockOptions() : *options;
}

bool validModeratorTypeIndex(int index) {
    return index >= 0 && index < static_cast<int>(moderatorTypes().size());
}

bool validReflectorTypeIndex(int index) {
    return index >= 0 && index < static_cast<int>(reflectorTypes().size());
}

int strongestModeratorType(const std::vector<int>& moderatorTypeIndices) {
    int bestType = -1;
    int bestFlux = std::numeric_limits<int>::min();
    for (int type : moderatorTypeIndices) {
        if (!validModeratorTypeIndex(type)) {
            continue;
        }
        const int flux = moderatorTypes().at(static_cast<size_t>(type)).fluxFactor;
        if (flux > bestFlux || (flux == bestFlux && (bestType < 0 || type < bestType))) {
            bestType = type;
            bestFlux = flux;
        }
    }
    return bestType;
}

void fillSupportBlocks(Grid& grid, const SupportBlockOptions* supportOptions) {
    NCFR_PERF_COUNT(fillSupportCalls);
    NCFR_PERF_SCOPE(fillSupportNs);
    const SupportBlockOptions& support = effectiveSupportBlockOptions(supportOptions);
    const int moderatorType = strongestModeratorType(support.moderatorTypeIndices);
    for (const Pos& pos : grid.interiorPositions()) {
        Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind != BlockKind::Empty) {
            continue;
        }
        if (moderatorType >= 0 &&
            (isBetweenFuelCells(grid, pos, 0) || isBetweenFuelCells(grid, pos, 1) ||
             isBetweenFuelCells(grid, pos, 2))) {
            block = {BlockKind::Moderator, moderatorType};
        }
    }

    for (const Pos& pos : grid.interiorPositions()) {
        Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind != BlockKind::Empty) {
            continue;
        }
        const int cells = adjacentCells(grid, pos);
        if (cells >= 3) {
            block = {BlockKind::Sink, 31};
        } else if (cells >= 2) {
            block = {BlockKind::Sink, 21};
        } else if (cells >= 1) {
            block = {BlockKind::Sink, 0};
        }
    }

    bool changed = true;
    for (int pass = 0; pass < 8 && changed; ++pass) {
        changed = false;
        StateVector validSinks;
        StateVector functionalCells;
        StateVector activeModerators;
        StateVector activeReflectors;
        StateVector functionalShields;
        StateVector functionalIrradiators;
        RuleContext context = optimisticRuleContext(grid, validSinks, functionalCells, activeModerators, activeReflectors,
                                                    functionalShields, functionalIrradiators);
        for (const Pos& pos : grid.interiorPositions()) {
            Block& block = grid.at(pos.x, pos.y, pos.z);
            if (block.kind != BlockKind::Empty) {
                continue;
            }
            int bestType = -1;
            int bestCooling = -1;
            for (const SinkType& sink : sinkTypes()) {
                if (sink.cooling <= bestCooling) {
                    continue;
                }
                block = {BlockKind::Sink, sink.index};
                if (optimisticSinkValidAt(grid, pos, context)) {
                    bestType = sink.index;
                    bestCooling = sink.cooling;
                }
            }
            if (bestType >= 0) {
                block = {BlockKind::Sink, bestType};
                changed = true;
            } else {
                block = {BlockKind::Empty, -1};
            }
        }
    }
    removeUnclusteredCornerManaDustSinks(grid);
}

std::vector<Pos> fuelPositionsInGrid(const Grid& grid);
bool allSourcesTargetFuel(const Grid& grid);

bool allSourcesTargetFuel(const Grid& grid) {
    NCFR_PERF_COUNT(allSourcesTargetFuelCalls);
    NCFR_PERF_SCOPE(allSourcesTargetFuelNs);
    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                if (grid.at(x, y, z).kind == BlockKind::Source &&
                    sourcePrimingTargetIndex(grid, {x, y, z}) < 0) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool hasNoEmptyInteriorPlane(const Grid& grid) {
    NCFR_PERF_COUNT(hasNoEmptyInteriorPlaneCalls);
    NCFR_PERF_SCOPE(hasNoEmptyInteriorPlaneNs);
    for (int x = 1; x <= grid.internalA(); ++x) {
        bool hasBlock = false;
        for (int y = 1; y <= grid.internalB() && !hasBlock; ++y) {
            for (int z = 1; z <= grid.internalC(); ++z) {
                if (grid.at(x, y, z).kind != BlockKind::Empty) {
                    hasBlock = true;
                    break;
                }
            }
        }
        if (!hasBlock) {
            return false;
        }
    }

    for (int y = 1; y <= grid.internalB(); ++y) {
        bool hasBlock = false;
        for (int x = 1; x <= grid.internalA() && !hasBlock; ++x) {
            for (int z = 1; z <= grid.internalC(); ++z) {
                if (grid.at(x, y, z).kind != BlockKind::Empty) {
                    hasBlock = true;
                    break;
                }
            }
        }
        if (!hasBlock) {
            return false;
        }
    }

    for (int z = 1; z <= grid.internalC(); ++z) {
        bool hasBlock = false;
        for (int y = 1; y <= grid.internalB() && !hasBlock; ++y) {
            for (int x = 1; x <= grid.internalA(); ++x) {
                if (grid.at(x, y, z).kind != BlockKind::Empty) {
                    hasBlock = true;
                    break;
                }
            }
        }
        if (!hasBlock) {
            return false;
        }
    }

    return true;
}

Grid compactEmptyInteriorPlanes(Grid grid) {
    NCFR_PERF_COUNT(compactInteriorPlanesCalls);
    NCFR_PERF_SCOPE(compactInteriorPlanesNs);

    std::vector<bool> keepX(static_cast<size_t>(grid.internalA() + 1), false);
    std::vector<bool> keepY(static_cast<size_t>(grid.internalB() + 1), false);
    std::vector<bool> keepZ(static_cast<size_t>(grid.internalC() + 1), false);

    for (const Pos& pos : grid.interiorPositions()) {
        if (grid.at(pos.x, pos.y, pos.z).kind == BlockKind::Empty) {
            continue;
        }
        keepX.at(static_cast<size_t>(pos.x)) = true;
        keepY.at(static_cast<size_t>(pos.y)) = true;
        keepZ.at(static_cast<size_t>(pos.z)) = true;
    }

    const int newA = static_cast<int>(std::count(keepX.begin(), keepX.end(), true));
    const int newB = static_cast<int>(std::count(keepY.begin(), keepY.end(), true));
    const int newC = static_cast<int>(std::count(keepZ.begin(), keepZ.end(), true));
    if (newA <= 0 || newB <= 0 || newC <= 0 ||
        (newA == grid.internalA() && newB == grid.internalB() && newC == grid.internalC())) {
        return grid;
    }

    std::vector<int> mapX(static_cast<size_t>(grid.internalA() + 1), 0);
    std::vector<int> mapY(static_cast<size_t>(grid.internalB() + 1), 0);
    std::vector<int> mapZ(static_cast<size_t>(grid.internalC() + 1), 0);
    for (int x = 1, next = 1; x <= grid.internalA(); ++x) {
        if (keepX.at(static_cast<size_t>(x))) {
            mapX.at(static_cast<size_t>(x)) = next++;
        }
    }
    for (int y = 1, next = 1; y <= grid.internalB(); ++y) {
        if (keepY.at(static_cast<size_t>(y))) {
            mapY.at(static_cast<size_t>(y)) = next++;
        }
    }
    for (int z = 1, next = 1; z <= grid.internalC(); ++z) {
        if (keepZ.at(static_cast<size_t>(z))) {
            mapZ.at(static_cast<size_t>(z)) = next++;
        }
    }

    Grid compacted = makeShell(newA, newB, newC);
    for (const Pos& pos : grid.interiorPositions()) {
        const Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::Empty) {
            continue;
        }
        compacted.at(mapX.at(static_cast<size_t>(pos.x)),
                     mapY.at(static_cast<size_t>(pos.y)),
                     mapZ.at(static_cast<size_t>(pos.z))) = block;
    }

    auto mappedCoordinate = [](int value, int oldInternalSize, int newInternalSize,
                               const std::vector<bool>& keep,
                               const std::vector<int>& map) -> std::optional<int> {
        if (value == 0) {
            return 0;
        }
        if (value == oldInternalSize + 1) {
            return newInternalSize + 1;
        }
        if (value >= 1 && value <= oldInternalSize && keep.at(static_cast<size_t>(value))) {
            return map.at(static_cast<size_t>(value));
        }
        return std::nullopt;
    };

    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                if (!grid.isBoundary(x, y, z)) {
                    continue;
                }
                const Block& block = grid.at(x, y, z);
                if (block.kind != BlockKind::Source) {
                    continue;
                }
                std::optional<int> compactedX =
                    mappedCoordinate(x, grid.internalA(), compacted.internalA(), keepX, mapX);
                std::optional<int> compactedY =
                    mappedCoordinate(y, grid.internalB(), compacted.internalB(), keepY, mapY);
                std::optional<int> compactedZ =
                    mappedCoordinate(z, grid.internalC(), compacted.internalC(), keepZ, mapZ);
                if (!compactedX.has_value() || !compactedY.has_value() || !compactedZ.has_value()) {
                    continue;
                }
                compacted.at(*compactedX, *compactedY, *compactedZ) = block;
            }
        }
    }

    return compacted;
}

std::vector<Pos> fuelPositionsInGrid(const Grid& grid) {
    std::vector<Pos> positions;
    for (const Pos& pos : grid.interiorPositions()) {
        if (grid.at(pos.x, pos.y, pos.z).kind == BlockKind::FuelCell) {
            positions.push_back(pos);
        }
    }
    return positions;
}

std::vector<Block> replacementBlocks(const SupportBlockOptions* supportOptions) {
    const SupportBlockOptions& support = effectiveSupportBlockOptions(supportOptions);
    std::vector<SinkType> sinks = sinkTypes();
    sinks.erase(std::remove_if(sinks.begin(), sinks.end(), [](const SinkType& sink) { return sink.cooling <= 0; }),
                sinks.end());
    std::sort(sinks.begin(), sinks.end(), [](const SinkType& lhs, const SinkType& rhs) {
        return lhs.cooling > rhs.cooling;
    });

    std::vector<Block> blocks;
    for (size_t i = 0; i < sinks.size() && i < 56; ++i) {
        blocks.push_back({BlockKind::Sink, sinks.at(i).index});
    }
    blocks.push_back({BlockKind::Shield, 1});
    blocks.push_back({BlockKind::Shield, 0});
    for (int moderatorType : support.moderatorTypeIndices) {
        if (validModeratorTypeIndex(moderatorType)) {
            blocks.push_back({BlockKind::Moderator, moderatorType});
        }
    }
    for (int reflectorType : support.reflectorTypeIndices) {
        if (validReflectorTypeIndex(reflectorType)) {
            blocks.push_back({BlockKind::Reflector, reflectorType});
        }
    }
    blocks.push_back({BlockKind::Empty, -1});
    return blocks;
}

bool isPreCompactRunnable(const FuelSimulation& sim);
bool restoreDirectionalFuelLines(Grid& grid, const BuildRequest& request, const std::vector<int>& sourceDirections,
                                 const std::vector<int>& reflectorDirections);

bool isSupportMutable(const Block& block) {
    return block.kind == BlockKind::Empty || block.kind == BlockKind::Sink || block.kind == BlockKind::Shield ||
           block.kind == BlockKind::Moderator || block.kind == BlockKind::Reflector;
}

bool isRequiredSupportBlock(const Grid& grid, const FuelSimulation& sim, int idx) {
    if (sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
        return true;
    }

    // Active moderators/reflectors support neutron lines without acting as heat-cluster conductors.
    const Block& block = grid.atIndex(idx);
    switch (block.kind) {
    case BlockKind::Moderator:
        return sim.activeModerators.at(static_cast<size_t>(idx));
    case BlockKind::Reflector:
        return sim.activeReflectors.at(static_cast<size_t>(idx));
    default:
        return false;
    }
}

int countFunctionalIrradiators(const FuelSimulation& sim) {
    return static_cast<int>(std::count(sim.functionalIrradiators.begin(), sim.functionalIrradiators.end(), true));
}

int countUsefulBlocks(const Grid& grid) {
    int count = 0;
    for (const Pos& pos : grid.interiorPositions()) {
        if (isFunctionalInterior(grid.at(pos.x, pos.y, pos.z).kind)) {
            ++count;
        }
    }
    return count;
}

std::vector<int> uniqueFuelIndicesInRequest(const BuildRequest& request) {
    std::vector<int> unique;
    unique.reserve(request.fuelIndices.size());
    for (int fuelIndex : request.fuelIndices) {
        if (std::find(unique.begin(), unique.end(), fuelIndex) == unique.end()) {
            unique.push_back(fuelIndex);
        }
    }
    return unique;
}

std::vector<Pos> fuelCellPortPositions(const Grid& grid) {
    std::vector<Pos> positions;
    positions.reserve(static_cast<size_t>(grid.volume()));
    auto appendIfCasing = [&](int x, int y, int z) {
        if (grid.isBoundary(x, y, z) && grid.at(x, y, z).kind == BlockKind::Casing) {
            positions.push_back({x, y, z});
        }
    };

    for (int z = 1; z < grid.depth() - 1; ++z) {
        for (int y = 1; y < grid.height() - 1; ++y) {
            appendIfCasing(0, y, z);
            appendIfCasing(grid.width() - 1, y, z);
        }
    }
    for (int z = 1; z < grid.depth() - 1; ++z) {
        for (int x = 1; x < grid.width() - 1; ++x) {
            appendIfCasing(x, 0, z);
            appendIfCasing(x, grid.height() - 1, z);
        }
    }
    for (int y = 1; y < grid.height() - 1; ++y) {
        for (int x = 1; x < grid.width() - 1; ++x) {
            appendIfCasing(x, y, 0);
            appendIfCasing(x, y, grid.depth() - 1);
        }
    }
    return positions;
}

void addFuelCellPorts(Grid& grid, const BuildRequest& request) {
    const std::vector<int> uniqueFuelIndices = uniqueFuelIndicesInRequest(request);
    const std::vector<Pos> portPositions = fuelCellPortPositions(grid);
    if (portPositions.size() < uniqueFuelIndices.size() * 2) {
        throw std::runtime_error("外壳空间不足，无法为每种燃料放置输入/输出燃料单元端口。");
    }

    size_t positionIndex = 0;
    for (int fuelIndex : uniqueFuelIndices) {
        const Pos inputPos = portPositions.at(positionIndex++);
        grid.at(inputPos.x, inputPos.y, inputPos.z) =
            {BlockKind::CellPort, fuelCellPortType(fuelIndex, FuelCellPortRole::Input)};

        const Pos outputPos = portPositions.at(positionIndex++);
        grid.at(outputPos.x, outputPos.y, outputPos.z) =
            {BlockKind::CellPort, fuelCellPortType(fuelIndex, FuelCellPortRole::Output)};
    }
}

void addIrradiatorPort(Grid& grid) {
    bool hasIrradiator = false;
    int portCount = 0;
    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                const Block& block = grid.at(x, y, z);
                hasIrradiator = hasIrradiator || block.kind == BlockKind::Irradiator;
                if (block.kind == BlockKind::IrradiatorPort) {
                    ++portCount;
                }
            }
        }
    }
    if (!hasIrradiator || portCount >= 2) {
        return;
    }

    for (const Pos& pos : fuelCellPortPositions(grid)) {
        if (portCount >= 2) {
            return;
        }
        const FuelCellPortRole role = portCount == 0 ? FuelCellPortRole::Input : FuelCellPortRole::Output;
        grid.at(pos.x, pos.y, pos.z) = {BlockKind::IrradiatorPort, irradiatorPortType(role)};
        ++portCount;
    }
    throw std::runtime_error("外壳空间不足，无法放置输入/输出辐照器端口。");
}

double totalIrradiatorFlux(const FuelSimulation& sim) {
    double total = 0.0;
    for (double flux : sim.irradiatorFluxByIndex) {
        total += flux;
    }
    return total;
}

CandidateScore scoreSimulation(const Grid& grid, const FuelSimulation& sim) {
    return {sim.compatible,
            hasSafeFuelFlux(grid, sim),
            sim.minClusterMargin,
            sim.disconnectedFunctionalBlocks,
            countFunctionalIrradiators(sim),
            countUsefulBlocks(grid),
            sim.cooling};
}

bool betterScore(const CandidateScore& lhs, const CandidateScore& rhs) {
    if (lhs.compatible != rhs.compatible) {
        return lhs.compatible;
    }
    if (lhs.safeFlux != rhs.safeFlux) {
        return lhs.safeFlux;
    }
    if (lhs.minCoolingMargin != rhs.minCoolingMargin) {
        return lhs.minCoolingMargin > rhs.minCoolingMargin;
    }
    if (lhs.disconnectedFunctionalBlocks != rhs.disconnectedFunctionalBlocks) {
        return lhs.disconnectedFunctionalBlocks < rhs.disconnectedFunctionalBlocks;
    }
    if (lhs.functionalIrradiators != rhs.functionalIrradiators) {
        return lhs.functionalIrradiators > rhs.functionalIrradiators;
    }
    if (lhs.cooling != rhs.cooling) {
        return lhs.cooling > rhs.cooling;
    }
    return lhs.usefulBlocks < rhs.usefulBlocks;
}

std::vector<Pos> improvementPositions(const Grid& grid, const FuelSimulation& sim, const ImproveOptions& options) {
    StateVector marked(static_cast<size_t>(grid.volume()), false);
    std::vector<Pos> positions;
    std::deque<std::pair<int, int>> queue;
    for (int idx = 0; idx < grid.volume(); ++idx) {
        if (!sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            continue;
        }
        marked.at(static_cast<size_t>(idx)) = true;
        queue.push_back({idx, 0});
    }
    while (!queue.empty()) {
        const auto [idx, distance] = queue.front();
        queue.pop_front();
        if (distance >= options.frontierRadius) {
            continue;
        }
        const int x = idx % grid.width();
        const int yz = idx / grid.width();
        const int y = yz % grid.height();
        const int z = yz / grid.height();
        grid.forEachNeighbor6({x, y, z}, [&](const Pos& n) {
            if (!grid.isInterior(n.x, n.y, n.z)) {
                return;
            }
            const int nIdx = grid.index(n.x, n.y, n.z);
            if (marked.at(static_cast<size_t>(nIdx))) {
                return;
            }
            marked.at(static_cast<size_t>(nIdx)) = true;
            if (isSupportMutable(grid.atIndex(nIdx)) && positions.size() < options.frontierLimit) {
                positions.push_back(n);
            }
            if (distance + 1 < options.frontierRadius) {
                queue.push_back({nIdx, distance + 1});
            }
        });
        if (positions.size() >= options.frontierLimit) {
            break;
        }
    }
    return positions;
}

Grid improveSupportBlocks(Grid grid, const std::atomic_bool* cancelRequested,
                          const ImproveOptions& options,
                          const SupportBlockOptions* supportOptions) {
    NCFR_PERF_COUNT(improveCalls);
    NCFR_PERF_SCOPE(improveNs);
    const std::vector<Block> blocks = replacementBlocks(supportOptions);
    for (int pass = 0; pass < options.maxPasses; ++pass) {
        throwIfCancelled(cancelRequested);
        NCFR_PERF_COUNT(improvePasses);
        FuelSimulation baseSim = simulateMixedFuel(grid);
        CandidateScore bestScore = scoreSimulation(grid, baseSim);
        std::vector<Pos> positions = improvementPositions(grid, baseSim, options);
        NCFR_PERF_ADD(improveFrontierPositions, positions.size());
        if (positions.empty()) {
            break;
        }

        bool found = false;
        Grid bestGrid = grid;
        for (const Pos& pos : positions) {
            throwIfCancelled(cancelRequested);
            const Block original = grid.at(pos.x, pos.y, pos.z);
            for (const Block& replacement : blocks) {
                throwIfCancelled(cancelRequested);
                if (original.kind == replacement.kind && original.type == replacement.type) {
                    continue;
                }
                Grid trial = grid;
                trial.at(pos.x, pos.y, pos.z) = replacement;
                removeUnclusteredCornerManaDustSinks(trial);
                NCFR_PERF_COUNT(improveTrials);
                FuelSimulation sim = simulateMixedFuel(trial);
                CandidateScore trialScore = scoreSimulation(trial, sim);
                if (betterScore(trialScore, bestScore)) {
                    bestScore = trialScore;
                    bestGrid = std::move(trial);
                    found = true;
                }
            }
        }
        if (!found) {
            break;
        }
        NCFR_PERF_COUNT(improveAcceptedPasses);
        grid = std::move(bestGrid);
        if (bestScore.compatible && bestScore.safeFlux && bestScore.minCoolingMargin >= 0 &&
            bestScore.disconnectedFunctionalBlocks == 0) {
            break;
        }
    }
    return grid;
}

OptimizationResult resultFromSimulation(Grid grid, const BuildRequest& request, const FuelSimulation& sim) {
    FuelSimulation finalSim = sim;
    const int originalInternalB = grid.internalB();
    Grid finalGrid = compactEmptyInteriorPlanes(std::move(grid));
    if (finalGrid.internalB() != originalInternalB) {
        finalSim = simulateMixedFuel(finalGrid);
        if (!isSafeOperatingSimulation(finalGrid, finalSim)) {
            throw std::runtime_error("高度压缩后方案不再满足安全运行判定。");
        }
    }

    addFuelCellPorts(finalGrid, request);
    addIrradiatorPort(finalGrid);
    OptimizationResult result(std::move(finalGrid), request);
    result.minCoolingMargin = finalSim.minClusterMargin;
    result.usefulBlocks = countUsefulBlocks(result.grid);
    result.disconnectedFunctionalBlocks = finalSim.disconnectedFunctionalBlocks;
    result.functionalIrradiators = countFunctionalIrradiators(finalSim);
    result.irradiatorFlux = totalIrradiatorFlux(finalSim);
    return result;
}

bool isAccepted(const Grid& grid, const FuelSimulation& sim) {
    return isSafeOperatingSimulation(grid, sim);
}

bool isPreCompactRunnable(const FuelSimulation& sim) {
    return sim.fuelCells > 0 && sim.runningCells == sim.fuelCells;
}

FinalizeFailureKind classifyFinalizationFailure(const Grid& grid, const FuelSimulation& sim,
                                                const BuildRequest& request) {
    (void)request;
    if (!isPreCompactRunnable(sim)) {
        return FinalizeFailureKind::NotRunnable;
    }
    if (!hasSafeFuelFlux(grid, sim)) {
        return FinalizeFailureKind::UnsafeFlux;
    }
    if (sim.disconnectedFunctionalBlocks != 0) {
        return FinalizeFailureKind::Disconnected;
    }
    if (!sim.compatible || sim.minClusterMargin < 0) {
        return FinalizeFailureKind::CoolingDeficit;
    }
    return FinalizeFailureKind::Structural;
}

FinalizeFailureKind finalizeFailureFromFuelRelation(const FuelRelationPrefilterResult& result) {
    switch (result.reason) {
    case FuelRelationRejectReason::NoSeed:
    case FuelRelationRejectReason::FuelNotRunnable:
        return FinalizeFailureKind::NotRunnable;
    case FuelRelationRejectReason::None:
    case FuelRelationRejectReason::NoFuelCells:
    case FuelRelationRejectReason::MissingFuelData:
        return FinalizeFailureKind::Structural;
    }
    return FinalizeFailureKind::Structural;
}

} // namespace ncfr::optimizer_detail
