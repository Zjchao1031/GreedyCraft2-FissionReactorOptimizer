#include "OptimizerDetail.h"

#include "FuelSpecialCases.h"
#include "NeutronRules.h"
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

struct EndStoneReflectorCandidate {
    Pos pos;
    int faceDirection = -1;
};

struct CarobbiiteReflectorCandidate {
    Pos pos;
    Pos endStonePos;
    int faceDirection = -1;
};

#ifndef NDEBUG
struct HighHeatPlacementFailureStats {
    size_t sinkTypeMissing = 0;
    size_t noCandidates = 0;
    size_t occupied = 0;
    size_t protectedPosition = 0;
    size_t requiredEndStoneInvalid = 0;
    size_t notRunnable = 0;
    size_t unsafeFlux = 0;
    size_t invalidSink = 0;
    size_t noHeatingCluster = 0;
    size_t alreadyConnected = 0;
    size_t noConnectionPath = 0;
    size_t connectionTrialInvalid = 0;
    size_t connected = 0;
};

std::string posLabel(const Pos& pos) {
    std::ostringstream os;
    os << "(" << pos.x << "," << pos.y << "," << pos.z << ")";
    return os.str();
}

const char* blockKindLabel(BlockKind kind) {
    switch (kind) {
        case BlockKind::Empty: return "Empty";
        case BlockKind::Casing: return "Casing";
        case BlockKind::Controller: return "Controller";
        case BlockKind::CellPort: return "CellPort";
        case BlockKind::IrradiatorPort: return "IrradiatorPort";
        case BlockKind::VentIn: return "VentIn";
        case BlockKind::VentOut: return "VentOut";
        case BlockKind::FuelCell: return "FuelCell";
        case BlockKind::Moderator: return "Moderator";
        case BlockKind::Reflector: return "Reflector";
        case BlockKind::Sink: return "Sink";
        case BlockKind::Conductor: return "Conductor";
        case BlockKind::Source: return "Source";
        case BlockKind::Shield: return "Shield";
        case BlockKind::Irradiator: return "Irradiator";
    }
    return "Unknown";
}

void appendHighHeatFailureStats(std::ostringstream& os,
                                const char* prefix,
                                const HighHeatPlacementFailureStats& stats) {
    os << " " << prefix << "MissingType=" << stats.sinkTypeMissing
       << " " << prefix << "NoCandidates=" << stats.noCandidates
       << " " << prefix << "Occupied=" << stats.occupied
       << " " << prefix << "Protected=" << stats.protectedPosition
       << " " << prefix << "RequiredEndStoneInvalid=" << stats.requiredEndStoneInvalid
       << " " << prefix << "NotRunnable=" << stats.notRunnable
       << " " << prefix << "UnsafeFlux=" << stats.unsafeFlux
       << " " << prefix << "InvalidSink=" << stats.invalidSink
       << " " << prefix << "NoHeatingCluster=" << stats.noHeatingCluster
       << " " << prefix << "AlreadyConnected=" << stats.alreadyConnected
       << " " << prefix << "NoConnectionPath=" << stats.noConnectionPath
       << " " << prefix << "ConnectionTrialInvalid=" << stats.connectionTrialInvalid
       << " " << prefix << "Connected=" << stats.connected;
}
#endif

bool isSpecialManaDustRequest(const BuildRequest& request) {
    return request.fuelIndices.size() == 1 &&
           usesSpecialManaDustCornerSinks(
               fuels().at(static_cast<size_t>(request.fuelIndices.front())));
}

bool hasSpecialManaDustCoolingDeficit(const FuelSimulation& sim) {
    return hasManaDustFallbackCoolingDeficit(sim.rawHeating - sim.cooling);
}

bool isHighHeatSingleFuelFallbackEligible(const BuildRequest& request, const FuelSimulation& sim) {
    if (request.fuelIndices.size() != 1) {
        return false;
    }
    const Fuel& fuel = fuels().at(static_cast<size_t>(request.fuelIndices.front()));
    const long long deficit = sim.rawHeating - sim.cooling;
    if (usesEndStoneOnlyReflectorCooling(fuel)) {
        return hasEndStoneFallbackCoolingDeficit(deficit);
    }
    if (usesCarobbiiteReflectorCooling(fuel)) {
        return hasEndStoneCarobbiiteFallbackCoolingDeficit(deficit);
    }
    if (usesSpecialManaDustCornerSinks(fuel)) {
        return hasCombinedHighHeatFallbackCoolingDeficit(deficit);
    }
    return false;
}

bool containsDirectionIndex(const std::vector<int>& indices, int index) {
    return std::find(indices.begin(), indices.end(), index) != indices.end();
}

bool isFullyReflectiveReflectorType(int reflectorType) {
    return reflectorType >= 0 &&
           reflectorType < static_cast<int>(reflectorTypes().size()) &&
           reflectorTypes().at(static_cast<size_t>(reflectorType)).reflectivity >= 1.0;
}

double estimatedLineFlux(const Fuel& fuel, int moderatorType, int moderatorCount, int reflectorType) {
    const auto& moderator = moderatorTypes().at(static_cast<size_t>(moderatorType));
    const auto& reflector = reflectorTypes().at(static_cast<size_t>(reflectorType));
    const double lineFlux = fuel.intrinsicFlux + moderator.fluxFactor * moderatorCount;
    return std::floor(2.0 * lineFlux * reflector.reflectivity);
}

bool fuelLineWithinReflectorReach(const FuelLineSpec& line) {
    return line.moderatorCount >= 1 && line.moderatorCount <= kMaxReflectorLineModerators;
}

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

Pos offset(const Pos& pos, const Direction& dir, int distance) {
    return {pos.x + dir.dx * distance, pos.y + dir.dy * distance, pos.z + dir.dz * distance};
}

Pos sourcePositionForDirection(const Grid& grid, const Pos& fuelPos, const Direction& dir) {
    Pos pos{fuelPos.x, fuelPos.y, fuelPos.z};
    if (dir.dx > 0) pos.x = grid.width() - 1;
    if (dir.dx < 0) pos.x = 0;
    if (dir.dy > 0) pos.y = grid.height() - 1;
    if (dir.dy < 0) pos.y = 0;
    if (dir.dz > 0) pos.z = grid.depth() - 1;
    if (dir.dz < 0) pos.z = 0;
    return pos;
}

void enumerateDirectionCombinations(int start, int remaining, std::vector<int>& current,
                                    std::vector<std::vector<int>>& combinations) {
    if (remaining == 0) {
        combinations.push_back(current);
        return;
    }
    for (int index = start; index <= static_cast<int>(kSourceDirections.size()) - remaining; ++index) {
        current.push_back(index);
        enumerateDirectionCombinations(index + 1, remaining - 1, current, combinations);
        current.pop_back();
    }
}

std::vector<std::vector<int>> sourceDirectionCombinations(int sourceCount) {
    std::vector<std::vector<int>> combinations;
    std::vector<int> current;
    enumerateDirectionCombinations(0, sourceCount, current, combinations);
    return combinations;
}

std::vector<Dimension> singleFuelSearchDimensions() {
    return {{kMaxSize, kMaxSize, kMaxSize}};
}

bool placeDirectionalSources(Grid& grid, const BuildRequest& request, const Pos& fuelPos,
                             const std::vector<int>& sourceDirections) {
    if (static_cast<int>(sourceDirections.size()) != requiredSourceCountForFuels(request)) {
        return false;
    }
    for (int sourceDirection : sourceDirections) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(sourceDirection));
        const Pos sourcePos = sourcePositionForDirection(grid, fuelPos, dir);
        if (!grid.isBoundary(sourcePos.x, sourcePos.y, sourcePos.z)) {
            return false;
        }
        Block& block = grid.at(sourcePos.x, sourcePos.y, sourcePos.z);
        if (block.kind != BlockKind::Casing && block.kind != BlockKind::Source) {
            return false;
        }
        block = {BlockKind::Source, -1};
    }
    return allSourcesTargetFuel(grid);
}

void markProtected(StateVector& protectedPositions, const Grid& grid, const Pos& pos) {
    if (grid.inBounds(pos.x, pos.y, pos.z)) {
        protectedPositions.at(static_cast<size_t>(grid.index(pos.x, pos.y, pos.z))) = true;
    }
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

bool isFullyReflectiveReflector(const Block& block) {
    return block.kind == BlockKind::Reflector && block.type >= 0 &&
           reflectorTypes().at(static_cast<size_t>(block.type)).reflectivity >= 1.0;
}

Block sourceLineReplacementBlock(const BuildRequest& request) {
    int bestType = -1;
    double bestReflectivity = -1.0;
    for (int type : request.selectedReflectorTypeIndices) {
        const double reflectivity = reflectorTypes().at(static_cast<size_t>(type)).reflectivity;
        if (reflectivity >= 1.0) {
            continue;
        }
        if (reflectivity > bestReflectivity ||
            (reflectivity == bestReflectivity && (bestType < 0 || type < bestType))) {
            bestType = type;
            bestReflectivity = reflectivity;
        }
    }
    return bestType >= 0 ? Block{BlockKind::Reflector, bestType} : Block{BlockKind::Empty, -1};
}

void keepSourceLinesOpen(Grid& grid, const BuildRequest& request, const std::vector<int>& sourceDirections) {
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != 1) {
        return;
    }

    const Block replacement = sourceLineReplacementBlock(request);
    for (int sourceDirection : sourceDirections) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(sourceDirection));
        Pos pos = sourcePositionForDirection(grid, fuelPositions.front(), dir);
        while (grid.inBounds(pos.x, pos.y, pos.z)) {
            pos.x -= dir.dx;
            pos.y -= dir.dy;
            pos.z -= dir.dz;
            if (!grid.inBounds(pos.x, pos.y, pos.z) ||
                (pos.x == fuelPositions.front().x && pos.y == fuelPositions.front().y &&
                 pos.z == fuelPositions.front().z)) {
                break;
            }
            Block& block = grid.at(pos.x, pos.y, pos.z);
            if (isFullyReflectiveReflector(block)) {
                block = replacement;
            }
        }
    }
}

bool restoreDirectionalFuelLines(Grid& grid, const BuildRequest& request, const std::vector<int>& sourceDirections,
                                 const std::vector<FuelLineSpec>& fuelLines) {
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != 1) {
        return false;
    }

    const Pos fuelPos = fuelPositions.front();
    for (const FuelLineSpec& line : fuelLines) {
        if (!fuelLineWithinReflectorReach(line)) {
            return false;
        }
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(line.direction));
        for (int distance = 1; distance <= line.moderatorCount; ++distance) {
            const Pos moderatorPos = offset(fuelPos, dir, distance);
            if (!grid.isInterior(moderatorPos.x, moderatorPos.y, moderatorPos.z)) {
                return false;
            }
            grid.at(moderatorPos.x, moderatorPos.y, moderatorPos.z) = {BlockKind::Moderator, line.moderatorType};
        }

        const Pos reflectorPos = offset(fuelPos, dir, line.moderatorCount + 1);
        if (!grid.isInterior(reflectorPos.x, reflectorPos.y, reflectorPos.z)) {
            return false;
        }
        grid.at(reflectorPos.x, reflectorPos.y, reflectorPos.z) = {BlockKind::Reflector, line.reflectorType};
    }

    keepSourceLinesOpen(grid, request, sourceDirections);
    return placeDirectionalSources(grid, request, fuelPos, sourceDirections);
}

bool protectedPositionAt(const StateVector* protectedPositions, const Grid& grid, const Pos& pos) {
    return protectedPositions != nullptr &&
           static_cast<size_t>(grid.index(pos.x, pos.y, pos.z)) < protectedPositions->size() &&
           protectedPositions->at(static_cast<size_t>(grid.index(pos.x, pos.y, pos.z)));
}

struct HeatingClusterInfo {
    std::vector<Pos> blocks;
};

struct ConductorBridgeResult {
    Grid grid;
    FuelSimulation sim;
    bool attempted = false;
    bool success = false;
    int clusterCount = 0;
    int conductorsAdded = 0;
    std::string reason;
};

bool canAttemptConductorBridge(const Grid& grid, const FuelSimulation& sim) {
    return isPreCompactRunnable(sim) && hasSafeFuelFlux(grid, sim) && sim.minClusterMargin >= 0 &&
           sim.disconnectedFunctionalBlocks != 0;
}

std::vector<HeatingClusterInfo> heatingClusters(const Grid& grid, const FuelSimulation& sim) {
    std::vector<HeatingClusterInfo> clusters;
    StateVector visited(static_cast<size_t>(grid.volume()), false);
    for (int start = 0; start < grid.volume(); ++start) {
        if (visited.at(static_cast<size_t>(start)) || !sim.heatingClusterBlocks.at(static_cast<size_t>(start))) {
            continue;
        }
        HeatingClusterInfo cluster;
        bool hasHeating = false;
        std::deque<int> queue;
        queue.push_back(start);
        visited.at(static_cast<size_t>(start)) = true;
        while (!queue.empty()) {
            const int idx = queue.front();
            queue.pop_front();
            const Pos pos{idx % grid.width(), (idx / grid.width()) % grid.height(), idx / (grid.width() * grid.height())};
            cluster.blocks.push_back(pos);

            const Block& block = grid.atIndex(idx);
            if ((block.kind == BlockKind::FuelCell && sim.functionalCells.at(static_cast<size_t>(idx))) ||
                (block.kind == BlockKind::Shield && sim.functionalShields.at(static_cast<size_t>(idx))) ||
                (block.kind == BlockKind::Irradiator && sim.functionalIrradiators.at(static_cast<size_t>(idx)))) {
                hasHeating = true;
            }

            grid.forEachNeighbor6(pos, [&](const Pos& n) {
                if (!grid.isInterior(n.x, n.y, n.z)) {
                    return;
                }
                const int nIdx = grid.index(n.x, n.y, n.z);
                if (!visited.at(static_cast<size_t>(nIdx)) &&
                    sim.heatingClusterBlocks.at(static_cast<size_t>(nIdx))) {
                    visited.at(static_cast<size_t>(nIdx)) = true;
                    queue.push_back(nIdx);
                }
            });
        }
        if (hasHeating) {
            clusters.push_back(std::move(cluster));
        }
    }
    return clusters;
}

bool isBridgeMutableBlock(const Block& block) {
    return block.kind == BlockKind::Empty || isSupportMutable(block);
}

bool canBridgeThrough(const Grid& grid, int idx, const StateVector* protectedPositions,
                      const StateVector& targetMask) {
    if (targetMask.at(static_cast<size_t>(idx))) {
        return true;
    }
    if (protectedPositions != nullptr && static_cast<size_t>(idx) < protectedPositions->size() &&
        protectedPositions->at(static_cast<size_t>(idx))) {
        return false;
    }
    return isBridgeMutableBlock(grid.atIndex(idx));
}

std::optional<std::vector<Pos>> shortestConductorPath(
    const Grid& grid, const std::vector<Pos>& starts, const StateVector& targetMask,
    const StateVector* protectedPositions, const std::function<bool(const Pos&)>& targetReached,
    const std::atomic_bool* cancelRequested) {
    StateVector visited(static_cast<size_t>(grid.volume()), false);
    std::vector<int> previous(static_cast<size_t>(grid.volume()), -1);
    std::deque<int> queue;
    for (const Pos& start : starts) {
        if (!grid.isInterior(start.x, start.y, start.z)) {
            continue;
        }
        const int idx = grid.index(start.x, start.y, start.z);
        if (visited.at(static_cast<size_t>(idx))) {
            continue;
        }
        visited.at(static_cast<size_t>(idx)) = true;
        queue.push_back(idx);
    }

    while (!queue.empty()) {
        throwIfCancelled(cancelRequested);
        const int idx = queue.front();
        queue.pop_front();
        const Pos pos{idx % grid.width(), (idx / grid.width()) % grid.height(), idx / (grid.width() * grid.height())};
        if (targetReached(pos)) {
            std::vector<Pos> path;
            for (int cur = idx; cur >= 0; cur = previous.at(static_cast<size_t>(cur))) {
                const Pos curPos{cur % grid.width(),
                                 (cur / grid.width()) % grid.height(),
                                 cur / (grid.width() * grid.height())};
                path.push_back(curPos);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        grid.forEachNeighbor6(pos, [&](const Pos& n) {
            if (!grid.isInterior(n.x, n.y, n.z)) {
                return;
            }
            const int nIdx = grid.index(n.x, n.y, n.z);
            if (visited.at(static_cast<size_t>(nIdx)) ||
                !canBridgeThrough(grid, nIdx, protectedPositions, targetMask)) {
                return;
            }
            visited.at(static_cast<size_t>(nIdx)) = true;
            previous.at(static_cast<size_t>(nIdx)) = idx;
            queue.push_back(nIdx);
        });
    }
    return std::nullopt;
}

int placeConductorsOnPath(Grid& grid, const std::vector<Pos>& path, const StateVector& targetMask,
                          const StateVector& preserveMask) {
    int added = 0;
    for (const Pos& pos : path) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        if (targetMask.at(static_cast<size_t>(idx)) || preserveMask.at(static_cast<size_t>(idx))) {
            continue;
        }
        Block& block = grid.atIndex(idx);
        if (block.kind != BlockKind::Conductor) {
            block = {BlockKind::Conductor, -1};
            ++added;
        }
    }
    return added;
}

StateVector maskForClusterBlocks(const Grid& grid, const std::vector<HeatingClusterInfo>& clusters,
                                 const std::vector<int>& indices) {
    StateVector mask(static_cast<size_t>(grid.volume()), false);
    for (int clusterIndex : indices) {
        for (const Pos& pos : clusters.at(static_cast<size_t>(clusterIndex)).blocks) {
            mask.at(static_cast<size_t>(grid.index(pos.x, pos.y, pos.z))) = true;
        }
    }
    return mask;
}

bool bridgeStillSafe(const Grid& grid, const FuelSimulation& sim) {
    return isPreCompactRunnable(sim) && hasSafeFuelFlux(grid, sim) && sim.minClusterMargin >= 0;
}

ConductorBridgeResult connectHeatingClustersWithConductors(Grid grid, const FuelSimulation& initialSim,
                                                           const StateVector* protectedPositions,
                                                           const std::atomic_bool* cancelRequested) {
    ConductorBridgeResult result{grid, initialSim, false, false, 0, 0, ""};
    if (!canAttemptConductorBridge(grid, initialSim)) {
        result.reason = "notNeeded";
        return result;
    }

    result.attempted = true;
    std::vector<HeatingClusterInfo> clusters = heatingClusters(grid, initialSim);
    result.clusterCount = static_cast<int>(clusters.size());
    if (clusters.empty()) {
        result.reason = "noHeatingClusters";
        return result;
    }

    const int mainCluster = 0;
    std::vector<int> connectedClusters{mainCluster};

    for (int clusterIndex = 0; clusterIndex < static_cast<int>(clusters.size()); ++clusterIndex) {
        throwIfCancelled(cancelRequested);
        if (clusterIndex == mainCluster) {
            continue;
        }
        StateVector targetMask = maskForClusterBlocks(grid, clusters, connectedClusters);
        const auto path = shortestConductorPath(
            grid, clusters.at(static_cast<size_t>(clusterIndex)).blocks, targetMask, protectedPositions,
            [&](const Pos& pos) {
                return targetMask.at(static_cast<size_t>(grid.index(pos.x, pos.y, pos.z)));
            }, cancelRequested);
        if (!path.has_value()) {
            result.reason = "clusterPathMissing";
            return result;
        }
        Grid trial = grid;
        const int added = placeConductorsOnPath(trial, *path, targetMask, initialSim.heatingClusterBlocks);
        FuelSimulation trialSim = simulateMixedFuel(trial);
        if (!bridgeStillSafe(trial, trialSim)) {
            result.reason = "clusterPathUnsafe";
            return result;
        }
        result.conductorsAdded += added;
        grid = std::move(trial);
        result.sim = std::move(trialSim);
        connectedClusters.push_back(clusterIndex);
    }

    result.grid = std::move(grid);
    result.sim = simulateMixedFuel(result.grid);
    result.success = isSearchAccepted(result.grid, result.sim);
    result.reason = result.success ? "success" : "finalNotAccepted";
    return result;
}

#ifndef NDEBUG
void logConductorBridgeCheckpoint(const char* reason, const Grid& grid, const FuelSimulation& sim,
                                  int clusterCount, int conductorsAdded) {
    std::ostringstream os;
    os << "reason=" << reason
       << " grid=" << gridInteriorLabel(grid)
       << " compatible=" << (sim.compatible ? 1 : 0)
       << " minMargin=" << sim.minClusterMargin
       << " disconnected=" << sim.disconnectedFunctionalBlocks
       << " clusters=" << clusterCount
       << " conductorsAdded=" << conductorsAdded
       << " rawHeating=" << sim.rawHeating
       << " cooling=" << sim.cooling;
    NCFR_PERF_CHECKPOINT("conductorBridge", os.str().c_str());
}
#endif

std::vector<FuelLineSpec> singleFuelLineOptions(const Fuel& fuel, const BuildRequest& request,
                                                const std::vector<int>& sourceDirections,
                                                int direction) {
    std::vector<FuelLineSpec> options;
    for (int reflectorType : request.selectedReflectorTypeIndices) {
        if (containsDirectionIndex(sourceDirections, direction) &&
            isFullyReflectiveReflectorType(reflectorType)) {
            continue;
        }
        for (int moderatorType : request.selectedModeratorTypeIndices) {
            for (int moderators = 1; moderators <= kMaxReflectorLineModerators; ++moderators) {
                const double flux = estimatedLineFlux(fuel, moderatorType, moderators, reflectorType);
                if (flux > 2.0 * fuel.criticality + kFluxEpsilon) {
                    continue;
                }
                options.push_back({direction, moderators, moderatorType, reflectorType, flux});
            }
        }
    }
    std::sort(options.begin(), options.end(), [](const FuelLineSpec& lhs, const FuelLineSpec& rhs) {
        if (lhs.moderatorCount != rhs.moderatorCount) {
            return lhs.moderatorCount > rhs.moderatorCount;
        }
        if (lhs.estimatedFlux != rhs.estimatedFlux) {
            return lhs.estimatedFlux < rhs.estimatedFlux;
        }
        if (lhs.moderatorType != rhs.moderatorType) {
            return lhs.moderatorType < rhs.moderatorType;
        }
        if (lhs.reflectorType != rhs.reflectorType) {
            return lhs.reflectorType < rhs.reflectorType;
        }
        return lhs.direction < rhs.direction;
    });
    return options;
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

void pruneInactiveSupport(Grid& grid, const StateVector* protectedPositions) {
    FuelSimulation sim = simulateMixedFuel(grid);
    for (const Pos& pos : grid.interiorPositions()) {
        if (protectedPositionAt(protectedPositions, grid, pos)) {
            continue;
        }
        const int idx = grid.index(pos.x, pos.y, pos.z);
        Block& block = grid.atIndex(idx);
        if (block.kind == BlockKind::Sink && !sim.validSinks.at(static_cast<size_t>(idx))) {
            block = {BlockKind::Empty, -1};
            continue;
        }
        if (block.kind != BlockKind::Empty && isSupportMutable(block) &&
            !isRequiredSupportBlock(grid, sim, idx)) {
            block = {BlockKind::Empty, -1};
        }
    }
}

struct DirectionalCompactionPlan {
    std::vector<bool> keepX;
    std::vector<bool> keepY;
    std::vector<bool> keepZ;
    std::vector<int> mapX;
    std::vector<int> mapY;
    std::vector<int> mapZ;
    int newA = 0;
    int newB = 0;
    int newC = 0;
    bool keepConductors = false;
};

#ifndef NDEBUG
std::string keptPlaneRangeLabel(const std::vector<bool>& keep) {
    int first = -1;
    int last = -1;
    int count = 0;
    for (int coordinate = 1; coordinate < static_cast<int>(keep.size()); ++coordinate) {
        if (!keep.at(static_cast<size_t>(coordinate))) {
            continue;
        }
        if (first < 0) {
            first = coordinate;
        }
        last = coordinate;
        ++count;
    }
    if (first < 0) {
        return "none";
    }
    std::ostringstream os;
    os << first << ".." << last << "/" << count;
    return os.str();
}
#endif

std::optional<DirectionalCompactionPlan> buildDirectionalCompactionPlan(
    const Grid& grid, int paddingPlanes, bool keepConductors) {
    DirectionalCompactionPlan plan;
    plan.keepX.assign(static_cast<size_t>(grid.internalA() + 1), false);
    plan.keepY.assign(static_cast<size_t>(grid.internalB() + 1), false);
    plan.keepZ.assign(static_cast<size_t>(grid.internalC() + 1), false);
    plan.keepConductors = keepConductors;

    auto keepRange = [paddingPlanes](std::vector<bool>& keep, int center, int max) {
        const int begin = std::max(1, center - paddingPlanes);
        const int end = std::min(max, center + paddingPlanes);
        for (int value = begin; value <= end; ++value) {
            keep.at(static_cast<size_t>(value)) = true;
        }
    };

    for (const Pos& pos : grid.interiorPositions()) {
        const BlockKind kind = grid.at(pos.x, pos.y, pos.z).kind;
        if (kind == BlockKind::Empty || (!keepConductors && kind == BlockKind::Conductor)) {
            continue;
        }
        keepRange(plan.keepX, pos.x, grid.internalA());
        keepRange(plan.keepY, pos.y, grid.internalB());
        keepRange(plan.keepZ, pos.z, grid.internalC());
    }

    plan.newA = static_cast<int>(std::count(plan.keepX.begin(), plan.keepX.end(), true));
    plan.newB = static_cast<int>(std::count(plan.keepY.begin(), plan.keepY.end(), true));
    plan.newC = static_cast<int>(std::count(plan.keepZ.begin(), plan.keepZ.end(), true));
    if (plan.newA <= 0 || plan.newB <= 0 || plan.newC <= 0) {
        return std::nullopt;
    }

    plan.mapX.assign(static_cast<size_t>(grid.internalA() + 1), 0);
    plan.mapY.assign(static_cast<size_t>(grid.internalB() + 1), 0);
    plan.mapZ.assign(static_cast<size_t>(grid.internalC() + 1), 0);
    for (int x = 1, next = 1; x <= grid.internalA(); ++x) {
        if (plan.keepX.at(static_cast<size_t>(x))) {
            plan.mapX.at(static_cast<size_t>(x)) = next++;
        }
    }
    for (int y = 1, next = 1; y <= grid.internalB(); ++y) {
        if (plan.keepY.at(static_cast<size_t>(y))) {
            plan.mapY.at(static_cast<size_t>(y)) = next++;
        }
    }
    for (int z = 1, next = 1; z <= grid.internalC(); ++z) {
        if (plan.keepZ.at(static_cast<size_t>(z))) {
            plan.mapZ.at(static_cast<size_t>(z)) = next++;
        }
    }
    return plan;
}

std::optional<Grid> applyDirectionalCompactionPlan(
    const Grid& grid, const DirectionalCompactionPlan& plan,
    const BuildRequest& request, const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines) {
    if (plan.keepX.size() != static_cast<size_t>(grid.internalA() + 1) ||
        plan.keepY.size() != static_cast<size_t>(grid.internalB() + 1) ||
        plan.keepZ.size() != static_cast<size_t>(grid.internalC() + 1)) {
        return std::nullopt;
    }

    Grid compacted = makeShell(plan.newA, plan.newB, plan.newC);
    for (const Pos& pos : grid.interiorPositions()) {
        const Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::Empty) {
            continue;
        }
        if (!plan.keepX.at(static_cast<size_t>(pos.x)) ||
            !plan.keepY.at(static_cast<size_t>(pos.y)) ||
            !plan.keepZ.at(static_cast<size_t>(pos.z))) {
            if (!plan.keepConductors && block.kind == BlockKind::Conductor) {
                continue;
            }
            continue;
        }
        compacted.at(plan.mapX.at(static_cast<size_t>(pos.x)),
                     plan.mapY.at(static_cast<size_t>(pos.y)),
                     plan.mapZ.at(static_cast<size_t>(pos.z))) = block;
    }

    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(compacted);
    if (fuelPositions.size() != request.fuelIndices.size() ||
        !restoreDirectionalFuelLines(compacted, request, sourceDirections, fuelLines)) {
        return std::nullopt;
    }
    return compacted;
}

std::optional<Grid> compactInteriorPlanesPreservingSources(const Grid& grid, const BuildRequest& request,
                                                           const std::vector<int>& sourceDirections,
                                                           const std::vector<FuelLineSpec>& fuelLines,
                                                           int paddingPlanes = 0,
                                                           bool keepConductors = false) {
    NCFR_PERF_COUNT(compactInteriorPlanesCalls);
    NCFR_PERF_SCOPE(compactInteriorPlanesNs);
    const std::optional<DirectionalCompactionPlan> plan =
        buildDirectionalCompactionPlan(grid, paddingPlanes, keepConductors);
    if (!plan.has_value()) {
        return std::nullopt;
    }
#ifndef NDEBUG
    {
        std::ostringstream os;
        os << "old=" << gridInteriorLabel(grid)
           << " new=" << plan->newA << "x" << plan->newB << "x" << plan->newC
           << " keepX=" << keptPlaneRangeLabel(plan->keepX)
           << " keepY=" << keptPlaneRangeLabel(plan->keepY)
           << " keepZ=" << keptPlaneRangeLabel(plan->keepZ)
           << " padding=" << paddingPlanes
           << " keepConductors=" << (keepConductors ? 1 : 0);
        NCFR_PERF_CHECKPOINT("compaction.plan", os.str().c_str());
    }
#endif
    return applyDirectionalCompactionPlan(grid, *plan, request, sourceDirections, fuelLines);
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
                                              bool keepConductors = false) {
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

int sinkTypeForSourceName(const char* sourceName) {
    for (const SinkType& sink : sinkTypes()) {
        if (sink.sourceName == sourceName) {
            return sink.index;
        }
    }
    return -1;
}

int endStoneSinkType() {
    static const int type = [] {
        return sinkTypeForSourceName("end_stone");
    }();
    return type;
}

int carobbiiteSinkType() {
    static const int type = [] {
        return sinkTypeForSourceName("carobbiite");
    }();
    return type;
}

bool anyHeatingClusterBlock(const FuelSimulation& sim);

bool isEndStoneSink(const Block& block) {
    const int type = endStoneSinkType();
    return type >= 0 && block.kind == BlockKind::Sink && block.type == type;
}

bool isCarobbiiteSink(const Block& block) {
    const int type = carobbiiteSinkType();
    return type >= 0 && block.kind == BlockKind::Sink && block.type == type;
}

void markDirectionalLayoutProtected(StateVector& protectedPositions, const Grid& grid,
                                      const Pos& fuelPos,
                                      const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines) {
    markProtected(protectedPositions, grid, fuelPos);
    (void)sourceDirections;
    for (const FuelLineSpec& line : fuelLines) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(line.direction));
        for (int distance = 1; distance <= line.moderatorCount + 1; ++distance) {
            markProtected(protectedPositions, grid, offset(fuelPos, dir, distance));
        }
    }
}

void markOccupiedInteriorProtected(StateVector& protectedPositions,
                                   const Grid& grid) {
    for (const Pos& pos : grid.interiorPositions()) {
        if (grid.at(pos.x, pos.y, pos.z).kind != BlockKind::Empty) {
            markProtected(protectedPositions, grid, pos);
        }
    }
}

std::optional<std::vector<EndStoneReflectorCandidate>> endStoneReflectorSinkCandidates(
    const Grid& grid, const Pos& fuelPos,
    const std::vector<FuelLineSpec>& fuelLines) {
    if (endStoneSinkType() < 0 || fuelLines.size() != 1) {
        return std::nullopt;
    }

    std::vector<EndStoneReflectorCandidate> candidates;
    for (const FuelLineSpec& line : fuelLines) {
        const Direction& lineDirection =
            kSourceDirections.at(static_cast<size_t>(line.direction));
        const Pos reflectorPos =
            offset(fuelPos, lineDirection, line.moderatorCount + 1);
        if (!grid.isInterior(reflectorPos.x, reflectorPos.y, reflectorPos.z) ||
            grid.at(reflectorPos.x, reflectorPos.y, reflectorPos.z).kind !=
                BlockKind::Reflector) {
            return std::nullopt;
        }

        for (int faceDirectionIndex = 0;
             faceDirectionIndex < static_cast<int>(kSourceDirections.size());
             ++faceDirectionIndex) {
            const Direction& faceDirection =
                kSourceDirections.at(static_cast<size_t>(faceDirectionIndex));
            if (faceDirection.dx == -lineDirection.dx &&
                faceDirection.dy == -lineDirection.dy &&
                faceDirection.dz == -lineDirection.dz) {
                continue;
            }
            const Pos sinkPos = offset(reflectorPos, faceDirection, 1);
            if (!grid.isInterior(sinkPos.x, sinkPos.y, sinkPos.z)) {
                continue;
            }
            if (std::none_of(candidates.begin(), candidates.end(),
                             [&](const EndStoneReflectorCandidate& existing) {
                    return existing.pos.x == sinkPos.x &&
                           existing.pos.y == sinkPos.y &&
                           existing.pos.z == sinkPos.z;
                })) {
                candidates.push_back({sinkPos, faceDirectionIndex});
            }
        }
    }
    return candidates;
}

std::vector<CarobbiiteReflectorCandidate> carobbiiteReflectorSinkCandidates(
    const Grid& grid,
    const std::vector<FuelLineSpec>& fuelLines,
    const std::vector<EndStoneReflectorCandidate>& endStoneCandidates) {
    if (carobbiiteSinkType() < 0 || fuelLines.size() != 1) {
        return {};
    }

    const int lineDirectionIndex = fuelLines.front().direction;
    const Direction& lineDirection =
        kSourceDirections.at(static_cast<size_t>(lineDirectionIndex));
    std::vector<CarobbiiteReflectorCandidate> candidates;
    for (const EndStoneReflectorCandidate& endStoneCandidate :
         endStoneCandidates) {
        if (endStoneCandidate.faceDirection == lineDirectionIndex) {
            continue;
        }
        const Pos sinkPos = offset(endStoneCandidate.pos, lineDirection, -1);
        if (!grid.isInterior(sinkPos.x, sinkPos.y, sinkPos.z)) {
            continue;
        }
        if (std::none_of(candidates.begin(), candidates.end(),
                         [&](const CarobbiiteReflectorCandidate& existing) {
                return samePos(existing.pos, sinkPos);
            })) {
            candidates.push_back(
                {sinkPos, endStoneCandidate.pos,
                 endStoneCandidate.faceDirection});
        }
    }
    return candidates;
}

bool tryPlaceCarobbiiteSink(
    Grid& grid, FuelSimulation& currentSim,
    StateVector& protectedPositions,
    const CarobbiiteReflectorCandidate& candidate
#ifndef NDEBUG
    , HighHeatPlacementFailureStats* debugStats = nullptr
#endif
) {
    const int sinkType = carobbiiteSinkType();
    if (sinkType < 0) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->sinkTypeMissing;
#endif
        return false;
    }
    const int sinkIdx =
        grid.index(candidate.pos.x, candidate.pos.y, candidate.pos.z);
    if (protectedPositions.at(static_cast<size_t>(sinkIdx))) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->protectedPosition;
#endif
        return false;
    }
    if (grid.atIndex(sinkIdx).kind != BlockKind::Empty) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->occupied;
#endif
        return false;
    }
    const int endStoneIdx = grid.index(candidate.endStonePos.x,
                                      candidate.endStonePos.y,
                                      candidate.endStonePos.z);
    if (!currentSim.validSinks.at(static_cast<size_t>(endStoneIdx))) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->requiredEndStoneInvalid;
#endif
        return false;
    }

    Grid trial = grid;
    trial.atIndex(sinkIdx) = {BlockKind::Sink, sinkType};
    FuelSimulation trialSim = simulateMixedFuel(trial);
    if (!isPreCompactRunnable(trialSim)) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->notRunnable;
#endif
        return false;
    }
    if (!hasSafeFuelFlux(trial, trialSim)) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->unsafeFlux;
#endif
        return false;
    }
    if (!trialSim.validSinks.at(static_cast<size_t>(sinkIdx))) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->invalidSink;
#endif
        return false;
    }

    grid = std::move(trial);
    currentSim = std::move(trialSim);
    markProtected(protectedPositions, grid, candidate.pos);
    return true;
}

void removeCarobbiiteSinksForEndStone(
    Grid& grid, StateVector& protectedPositions,
    std::vector<int>& placedCarobbiiteFaces,
    const std::vector<CarobbiiteReflectorCandidate>& carobbiiteCandidates,
    const Pos& endStonePos) {
    for (const CarobbiiteReflectorCandidate& candidate :
         carobbiiteCandidates) {
        if (!samePos(candidate.endStonePos, endStonePos)) {
            continue;
        }
        const int sinkIdx =
            grid.index(candidate.pos.x, candidate.pos.y, candidate.pos.z);
        if (isCarobbiiteSink(grid.atIndex(sinkIdx))) {
            grid.atIndex(sinkIdx) = {BlockKind::Empty, -1};
            protectedPositions.at(static_cast<size_t>(sinkIdx)) = false;
        }
        placedCarobbiiteFaces.erase(
            std::remove(placedCarobbiiteFaces.begin(),
                        placedCarobbiiteFaces.end(),
                        candidate.faceDirection),
            placedCarobbiiteFaces.end());
    }
}

bool tryConnectSpecialSinkToHeatingCluster(
    Grid& grid, FuelSimulation& currentSim, const Pos& sinkPos,
    StateVector& protectedPositions, const std::atomic_bool* cancelRequested,
    const SimulationOptions& simulationOptions = {}
#ifndef NDEBUG
    , HighHeatPlacementFailureStats* debugStats = nullptr
#endif
) {
    if (!anyHeatingClusterBlock(currentSim)) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->noHeatingCluster;
#endif
        return false;
    }
    throwIfCancelled(cancelRequested);
    const int sinkIdx = grid.index(sinkPos.x, sinkPos.y, sinkPos.z);
    if (!currentSim.validSinks.at(static_cast<size_t>(sinkIdx))) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->invalidSink;
#endif
        return false;
    }
    if (currentSim.heatingClusterBlocks.at(static_cast<size_t>(sinkIdx))) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->alreadyConnected;
#endif
        return true;
    }

    const StateVector targetMask = currentSim.heatingClusterBlocks;
    const auto path = shortestConductorPath(
        grid, {sinkPos}, targetMask, &protectedPositions,
        [&](const Pos& pos) {
            return targetMask.at(
                static_cast<size_t>(grid.index(pos.x, pos.y, pos.z)));
        },
        cancelRequested);
    if (!path.has_value()) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->noConnectionPath;
#endif
        return false;
    }

    Grid trial = grid;
    placeConductorsOnPath(trial, *path, targetMask, protectedPositions);
    FuelSimulation trialSim = simulateMixedFuel(trial, simulationOptions);
    if (!isPreCompactRunnable(trialSim) ||
        !hasSafeFuelFlux(trial, trialSim) ||
        !trialSim.validSinks.at(static_cast<size_t>(sinkIdx)) ||
        !trialSim.heatingClusterBlocks.at(static_cast<size_t>(sinkIdx))) {
#ifndef NDEBUG
        if (debugStats != nullptr) ++debugStats->connectionTrialInvalid;
#endif
        return false;
    }
    grid = std::move(trial);
    currentSim = std::move(trialSim);
    for (const Pos& pathPos : *path) {
        if (grid.at(pathPos.x, pathPos.y, pathPos.z).kind ==
            BlockKind::Conductor) {
            markProtected(protectedPositions, grid, pathPos);
        }
    }
#ifndef NDEBUG
    if (debugStats != nullptr) ++debugStats->connected;
#endif
    return true;
}

bool connectSpecialSinksToHeatingCluster(
    Grid& grid, FuelSimulation& currentSim, const std::vector<Pos>& sinkPositions,
    StateVector& protectedPositions, const std::atomic_bool* cancelRequested,
    const SimulationOptions& simulationOptions = {}
#ifndef NDEBUG
    , HighHeatPlacementFailureStats* debugStats = nullptr
#endif
) {
    for (const Pos& sinkPos : sinkPositions) {
        if (!tryConnectSpecialSinkToHeatingCluster(
                grid, currentSim, sinkPos, protectedPositions,
                cancelRequested, simulationOptions
#ifndef NDEBUG
                , debugStats
#endif
                )) {
            return false;
        }
    }
    return true;
}

std::vector<Pos> placedSpecialCoolingSinkPositions(const Grid& grid) {
    std::vector<Pos> positions;
    const int endStoneType = endStoneSinkType();
    const int carobbiiteType = carobbiiteSinkType();
    const auto appendMatchingSinks =
        [&](const auto& matches) {
            for (const Pos& pos : grid.interiorPositions()) {
                const Block& block =
                    grid.at(pos.x, pos.y, pos.z);
                if (block.kind == BlockKind::Sink &&
                    matches(block)) {
                    positions.push_back(pos);
                }
            }
        };

    appendMatchingSinks(
        [endStoneType](const Block& block) {
            return block.type == endStoneType;
        });
    appendMatchingSinks(
        [carobbiiteType](const Block& block) {
            return block.type == carobbiiteType;
        });
    appendMatchingSinks(
        [](const Block& block) {
            return isManaDustSink(block);
        });
    return positions;
}

bool hasFunctionalEndStoneSinks(
    const Grid& grid, const FuelSimulation& sim,
    const std::vector<FuelLineSpec>& fuelLines,
    const std::vector<int>& placedFaceDirections) {
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != 1) {
        return false;
    }
    const auto candidates =
        endStoneReflectorSinkCandidates(grid, fuelPositions.front(), fuelLines);
    if (!candidates.has_value()) {
        return false;
    }

    for (int faceDirection : placedFaceDirections) {
        const auto candidate = std::find_if(
            candidates->begin(), candidates->end(),
            [faceDirection](const EndStoneReflectorCandidate& value) {
                return value.faceDirection == faceDirection;
            });
        if (candidate == candidates->end()) {
            return false;
        }
        const Pos& pos = candidate->pos;
        if (!isEndStoneSink(grid.at(pos.x, pos.y, pos.z))) {
            return false;
        }
        const int idx = grid.index(pos.x, pos.y, pos.z);
        if (!sim.validSinks.at(static_cast<size_t>(idx)) ||
            !sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            return false;
        }
    }
    return !placedFaceDirections.empty();
}

bool hasFunctionalSpecialCarobbiiteSinks(
    const Grid& grid, const FuelSimulation& sim,
    const std::vector<FuelLineSpec>& fuelLines,
    const std::vector<int>& placedFaceDirections) {
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != 1) {
        return false;
    }
    const auto endStoneCandidates =
        endStoneReflectorSinkCandidates(grid, fuelPositions.front(), fuelLines);
    if (!endStoneCandidates.has_value()) {
        return false;
    }
    const std::vector<CarobbiiteReflectorCandidate> candidates =
        carobbiiteReflectorSinkCandidates(
            grid, fuelLines, *endStoneCandidates);

    for (int faceDirection : placedFaceDirections) {
        const auto candidate = std::find_if(
            candidates.begin(), candidates.end(),
            [faceDirection](const CarobbiiteReflectorCandidate& value) {
                return value.faceDirection == faceDirection;
            });
        if (candidate == candidates.end()) {
            return false;
        }
        if (!isEndStoneSink(grid.at(candidate->endStonePos.x,
                                    candidate->endStonePos.y,
                                    candidate->endStonePos.z)) ||
            !isCarobbiiteSink(grid.at(candidate->pos.x,
                                      candidate->pos.y,
                                      candidate->pos.z))) {
            return false;
        }
        const int idx = grid.index(candidate->pos.x, candidate->pos.y,
                                   candidate->pos.z);
        if (!sim.validSinks.at(static_cast<size_t>(idx)) ||
            !sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            return false;
        }
    }
    return !placedFaceDirections.empty();
}

#ifndef NDEBUG
void logHighHeatCoolingCheckpoint(const char* reason, const Grid& grid,
                                  const FuelSimulation& sim,
                                  size_t endStoneCandidates,
                                  size_t endStonePlaced,
                                  size_t endStoneOccupied,
                                  size_t endStoneFailed,
                                  size_t carobbiiteCandidates,
                                  size_t carobbiitePlaced,
                                  size_t carobbiiteFailed,
                                  size_t manaDustSinks) {
    std::ostringstream os;
    os << "reason=" << reason
       << " grid=" << gridInteriorLabel(grid)
       << " rawHeating=" << sim.rawHeating
       << " cooling=" << sim.cooling
       << " deficit=" << (sim.rawHeating - sim.cooling)
       << " minMargin=" << sim.minClusterMargin
       << " endStoneCandidates=" << endStoneCandidates
       << " endStonePlaced=" << endStonePlaced
       << " endStoneOccupied=" << endStoneOccupied
       << " endStoneFailed=" << endStoneFailed
       << " carobbiiteCandidates=" << carobbiiteCandidates
       << " carobbiitePlaced=" << carobbiitePlaced
       << " carobbiiteFailed=" << carobbiiteFailed
       << " manaDustSinks=" << manaDustSinks;
    NCFR_PERF_CHECKPOINT("highHeatCooling", os.str().c_str());
}

void logHighHeatPlacementFailures(
    const char* sinkName, const Grid& grid,
    const HighHeatPlacementFailureStats& stats,
    const std::string& detail = {}) {
    std::ostringstream os;
    os << "sink=" << sinkName
       << " grid=" << gridInteriorLabel(grid);
    appendHighHeatFailureStats(os, "fail", stats);
    if (!detail.empty()) {
        os << " detail=" << detail;
    }
    NCFR_PERF_CHECKPOINT("highHeatPlacementFailure", os.str().c_str());
}

void logHighHeatFinalReview(
    const char* reason, const Grid& grid, const FuelSimulation& sim,
    bool accepted, bool endStoneFunctional, bool carobbiiteFunctional,
    size_t endStoneFaces, size_t carobbiiteFaces) {
    std::ostringstream os;
    os << "reason=" << reason
       << " grid=" << gridInteriorLabel(grid)
       << " accepted=" << (accepted ? 1 : 0)
       << " compatible=" << (sim.compatible ? 1 : 0)
       << " safeFlux=" << (hasSafeFuelFlux(grid, sim) ? 1 : 0)
       << " disconnected=" << sim.disconnectedFunctionalBlocks
       << " rawHeating=" << sim.rawHeating
       << " cooling=" << sim.cooling
       << " minMargin=" << sim.minClusterMargin
       << " endStoneFaces=" << endStoneFaces
       << " endStoneFunctional=" << (endStoneFunctional ? 1 : 0)
       << " carobbiiteFaces=" << carobbiiteFaces
       << " carobbiiteFunctional=" << (carobbiiteFunctional ? 1 : 0);
    NCFR_PERF_CHECKPOINT("highHeatFinalReview", os.str().c_str());
}

void logDualFuelCoolingCheckpoint(
    const char* reason, const Grid& grid, const FuelSimulation& sim,
    long long initialDeficit, bool allowCarobbiite, bool allowManaDust,
    const std::string& detail = {}) {
    std::ostringstream os;
    os << "reason=" << reason
       << " grid=" << gridInteriorLabel(grid)
       << " rawHeating=" << sim.rawHeating
       << " cooling=" << sim.cooling
       << " deficit=" << (sim.rawHeating - sim.cooling)
       << " initialDeficit=" << initialDeficit
       << " minMargin=" << sim.minClusterMargin
       << " compatible=" << (sim.compatible ? 1 : 0)
       << " safeFlux=" << (hasSafeFuelFlux(grid, sim) ? 1 : 0)
       << " disconnected=" << sim.disconnectedFunctionalBlocks
       << " invalidSinks=" << (hasInvalidSinks(grid, sim) ? 1 : 0)
       << " allowCarobbiite=" << (allowCarobbiite ? 1 : 0)
       << " allowManaDust=" << (allowManaDust ? 1 : 0);
    if (!detail.empty()) {
        os << " detail=" << detail;
    }
    NCFR_PERF_CHECKPOINT("dualFuelCooling", os.str().c_str());
}

void logDualFuelSinkCheckpoint(
    const char* stage, int requestSlot, const Pos& pos, int faceDirection,
    const char* outcome, const Grid& grid,
    const std::string& detail = {},
    const HighHeatPlacementFailureStats* stats = nullptr) {
    std::ostringstream os;
    os << "stage=" << stage
       << " slot=" << requestSlot
       << " pos=" << posLabel(pos)
       << " faceDirection=" << faceDirection
       << " outcome=" << outcome
       << " grid=" << gridInteriorLabel(grid);
    if (stats != nullptr) {
        appendHighHeatFailureStats(os, "fail", *stats);
    }
    if (!detail.empty()) {
        os << " detail=" << detail;
    }
    NCFR_PERF_CHECKPOINT("dualFuelSink", os.str().c_str());
}

void logDualFuelFallbackCheckpoint(
    const char* reason, const Grid& grid, const FuelSimulation& sim,
    long long initialDeficit, const std::string& detail = {}) {
    std::ostringstream os;
    os << "reason=" << reason
       << " grid=" << gridInteriorLabel(grid)
       << " rawHeating=" << sim.rawHeating
       << " cooling=" << sim.cooling
       << " deficit=" << (sim.rawHeating - sim.cooling)
       << " initialDeficit=" << initialDeficit;
    if (!detail.empty()) {
        os << " detail=" << detail;
    }
    NCFR_PERF_CHECKPOINT("dualFuelFallback", os.str().c_str());
}
#endif

bool hasFunctionalSpecialManaDustCornerSinks(const Grid& grid, const FuelSimulation& sim) {
    if (grid.internalA() < 2 || grid.internalB() < 2 || grid.internalC() < 2 ||
        sim.validSinks.size() != static_cast<size_t>(grid.volume()) ||
        sim.heatingClusterBlocks.size() != static_cast<size_t>(grid.volume())) {
        return false;
    }
    for (const Pos& corner : interiorCornerPositions(grid)) {
        if (!isManaDustSink(grid.at(corner.x, corner.y, corner.z))) {
            return false;
        }
        const int idx = grid.index(corner.x, corner.y, corner.z);
        if (!sim.validSinks.at(static_cast<size_t>(idx)) ||
            !sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            return false;
        }
    }
    return true;
}

std::optional<Grid> padMixedFuelGridForSpecialCooling(
    const Grid& grid, const BuildRequest& request,
    const std::vector<FuelLayoutContext>& fuelContexts,
    std::vector<FuelLayoutContext>& paddedContexts
#ifndef NDEBUG
    , const char** debugFailure
#endif
);

bool anyHeatingClusterBlock(const FuelSimulation& sim) {
    return std::find(sim.heatingClusterBlocks.begin(), sim.heatingClusterBlocks.end(), 1U) !=
           sim.heatingClusterBlocks.end();
}

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
    std::vector<FuelLayoutContext> paddedContexts;
#ifndef NDEBUG
    const char* manaPaddingFailure = "unknown";
#endif
    std::optional<Grid> padded = padMixedFuelGridForSpecialCooling(
        grid, request, fuelContexts, paddedContexts
#ifndef NDEBUG
        , &manaPaddingFailure
#endif
        );
    if (!padded.has_value()) {
#ifndef NDEBUG
        NCFR_PERF_CHECKPOINT("manaDust.padding", manaPaddingFailure);
#endif
        return std::nullopt;
    }

    Grid specialGrid = std::move(*padded);
    StateVector specialProtected(
        static_cast<size_t>(specialGrid.volume()), false);
    markOccupiedInteriorProtected(specialProtected, specialGrid);
    const std::vector<Pos> corners =
        interiorCornerPositions(specialGrid);
    const std::vector<SourcePrimingTarget> paddedSourceTargets =
        sourcePrimingTargets(specialGrid);
#ifndef NDEBUG
    HighHeatPlacementFailureStats manaDustPlacementStats;
    HighHeatPlacementFailureStats manaDustConnectionStats;
#endif
    StateVector forcedValidSinks(
        static_cast<size_t>(specialGrid.volume()), false);
    const int manaType = manaDustSinkType();
    if (manaType < 0 || corners.size() != 8) {
#ifndef NDEBUG
        ++manaDustPlacementStats.sinkTypeMissing;
        logHighHeatPlacementFailures(
            "mana_dust_place", specialGrid, manaDustPlacementStats);
#endif
        return std::nullopt;
    }
    for (const Pos& corner : corners) {
        const int idx =
            specialGrid.index(corner.x, corner.y, corner.z);
        if (specialGrid.atIndex(idx).kind != BlockKind::Empty) {
#ifndef NDEBUG
            ++manaDustPlacementStats.occupied;
            logHighHeatPlacementFailures(
                "mana_dust_place", specialGrid,
                manaDustPlacementStats,
                "corner=" + posLabel(corner));
#endif
            return std::nullopt;
        }
        specialGrid.atIndex(idx) = {BlockKind::Sink, manaType};
        specialProtected.at(static_cast<size_t>(idx)) = true;
        forcedValidSinks.at(static_cast<size_t>(idx)) = true;
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

    std::optional<Grid> compacted =
        compactInteriorPlanesPreservingSources(
            specialGrid, request, sourceDirections, fuelLines, 0, true);
    if (!compacted.has_value()) {
        return std::nullopt;
    }
    FuelSimulation finalSim = simulateMixedFuel(*compacted);
    const bool compactedSafe =
        isSafeOperatingSimulation(*compacted, finalSim);
    if (!compactedSafe ||
        !hasFunctionalSpecialManaDustCornerSinks(*compacted, finalSim) ||
        !hasRequiredSources(*compacted, request)) {
#ifndef NDEBUG
        const WallConnectionResult wall =
            evaluateHeatingClusterWallConnections(*compacted, finalSim);
        std::ostringstream os;
        os << "grid=" << gridInteriorLabel(*compacted)
           << " accepted=" << (compactedSafe ? 1 : 0)
           << " cornersValid="
           << (hasFunctionalSpecialManaDustCornerSinks(*compacted, finalSim) ? 1 : 0)
           << " heatingClusters=" << wall.heatingClusters
           << " wallDisconnected=" << wall.disconnectedHeatingClusters;
        NCFR_PERF_CHECKPOINT("wallConnection.final", os.str().c_str());
#endif
        return std::nullopt;
    }
    OptimizationResult result =
        resultFromSimulation(std::move(*compacted), request, finalSim);
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

std::optional<Grid> padMixedFuelGridForSpecialCooling(
    const Grid& grid, const BuildRequest& request,
    const std::vector<FuelLayoutContext>& fuelContexts,
    std::vector<FuelLayoutContext>& paddedContexts
#ifndef NDEBUG
    , const char** debugFailure = nullptr
#endif
) {
#ifndef NDEBUG
    const auto fail = [debugFailure](const char* reason) {
        if (debugFailure != nullptr) {
            *debugFailure = reason;
        }
        return std::optional<Grid>{};
    };
#endif
    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int minZ = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int maxY = std::numeric_limits<int>::min();
    int maxZ = std::numeric_limits<int>::min();
    for (const Pos& pos : grid.interiorPositions()) {
        if (grid.at(pos.x, pos.y, pos.z).kind == BlockKind::Empty) {
            continue;
        }
        minX = std::min(minX, pos.x);
        minY = std::min(minY, pos.y);
        minZ = std::min(minZ, pos.z);
        maxX = std::max(maxX, pos.x);
        maxY = std::max(maxY, pos.y);
        maxZ = std::max(maxZ, pos.z);
    }
    if (minX > maxX || minY > maxY || minZ > maxZ) {
#ifndef NDEBUG
        return fail("emptyInterior");
#else
        return std::nullopt;
#endif
    }

    const int newA = maxX - minX + 3;
    const int newB = maxY - minY + 3;
    const int newC = maxZ - minZ + 3;
    if (newA > kMaxSize || newB > kMaxSize ||
        newC > kMaxSize) {
#ifndef NDEBUG
        return fail("paddedGridExceedsMaxSize");
#else
        return std::nullopt;
#endif
    }

    Grid padded = makeShell(newA, newB, newC);
    for (const Pos& pos : grid.interiorPositions()) {
        const Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::Empty) {
            continue;
        }
        padded.at(
            pos.x - minX + 2,
            pos.y - minY + 2,
            pos.z - minZ + 2) = block;
    }

    paddedContexts = fuelContexts;
    for (FuelLayoutContext& context : paddedContexts) {
        context.fuelPos = {
            context.fuelPos.x - minX + 2,
            context.fuelPos.y - minY + 2,
            context.fuelPos.z - minZ + 2,
        };
    }

    const std::vector<SourcePrimingTarget> oldTargets =
        sourcePrimingTargets(grid);
    for (size_t contextIndex = 0;
         contextIndex < fuelContexts.size(); ++contextIndex) {
        const FuelLayoutContext& oldContext =
            fuelContexts.at(contextIndex);
        const FuelLayoutContext& newContext =
            paddedContexts.at(contextIndex);
        if (oldContext.requestSlot < 0 ||
            oldContext.requestSlot >=
                static_cast<int>(request.fuelIndices.size())) {
#ifndef NDEBUG
            return fail("invalidRequestSlot");
#else
            return std::nullopt;
#endif
        }
        const Fuel& fuel = fuels().at(static_cast<size_t>(
            request.fuelIndices.at(
                static_cast<size_t>(oldContext.requestSlot))));
        if (fuel.selfPriming) {
            continue;
        }

        const int oldFuelIndex = grid.index(
            oldContext.fuelPos.x, oldContext.fuelPos.y,
            oldContext.fuelPos.z);
        const auto target = std::find_if(
            oldTargets.begin(), oldTargets.end(),
            [oldFuelIndex](const SourcePrimingTarget& value) {
                return value.targetIndex == oldFuelIndex;
            });
        if (target == oldTargets.end()) {
#ifndef NDEBUG
            return fail("sourceTargetMissing");
#else
            return std::nullopt;
#endif
        }

        int sourceDirection = -1;
        for (int directionIndex = 0;
             directionIndex <
             static_cast<int>(kSourceDirections.size());
             ++directionIndex) {
            const Pos expected = sourcePositionForDirection(
                grid, oldContext.fuelPos,
                kSourceDirections.at(
                    static_cast<size_t>(directionIndex)));
            if (samePos(expected, target->source)) {
                sourceDirection = directionIndex;
                break;
            }
        }
        if (sourceDirection < 0) {
#ifndef NDEBUG
            return fail("sourceDirectionUnknown");
#else
            return std::nullopt;
#endif
        }

        const Direction& direction = kSourceDirections.at(
            static_cast<size_t>(sourceDirection));
        const Pos sourcePos = sourcePositionForDirection(
            padded, newContext.fuelPos, direction);
        padded.at(sourcePos.x, sourcePos.y, sourcePos.z) = {
            BlockKind::Source, -1};
        if (sourcePrimingTargetIndex(padded, sourcePos) !=
            padded.index(
                newContext.fuelPos.x, newContext.fuelPos.y,
                newContext.fuelPos.z)) {
#ifndef NDEBUG
            return fail("sourceTargetChanged");
#else
            return std::nullopt;
#endif
        }
    }

    if (!hasRequiredSources(padded, request)) {
#ifndef NDEBUG
        return fail("requiredSourcesInvalid");
#else
        return std::nullopt;
#endif
    }
    return padded;
}

std::optional<Grid> tryMixedFuelSpecialCoolingFallback(
    Grid grid, const BuildRequest& request,
    const std::vector<FuelLayoutContext>& fuelContexts,
    const std::atomic_bool* cancelRequested) {
    FuelSimulation currentSim = simulateMixedFuel(grid);
    const long long initialDeficit =
        currentSim.rawHeating - currentSim.cooling;
    if (fuelContexts.empty()) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "rejected", grid, currentSim, initialDeficit,
            "emptyFuelContexts");
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
    if (currentSim.disconnectedFunctionalBlocks != 0) {
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
        return isSearchAccepted(grid, currentSim)
                   ? std::optional<Grid>(std::move(grid))
                   : std::nullopt;
    }
    if (initialDeficit > kDualFuelManaDustDeficitLimit) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "rejected", grid, currentSim, initialDeficit,
            "deficitExceedsManaDustLimit");
#endif
        return std::nullopt;
    }
    const bool allowCarobbiite =
        initialDeficit > kDualFuelEndStoneDeficitLimit;
    const bool allowManaDust =
        initialDeficit > kDualFuelCarobbiiteDeficitLimit;

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
            if (!isSearchAccepted(finalGrid, finalSim) ||
                hasInvalidSinks(finalGrid, finalSim) ||
                finalSim.disconnectedFunctionalBlocks != 0) {
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
    std::vector<FuelLayoutContext> paddedContexts;
#ifndef NDEBUG
    const char* manaPaddingFailure = "unknown";
#endif
    std::optional<Grid> padded = padMixedFuelGridForSpecialCooling(
        grid, request, fuelContexts, paddedContexts
#ifndef NDEBUG
        , &manaPaddingFailure
#endif
        );
    if (!padded.has_value()) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "manaDustPaddingFailed", grid, currentSim, initialDeficit,
            manaPaddingFailure);
#endif
        return std::nullopt;
    }

    Grid manaGrid = std::move(*padded);
    StateVector manaProtected(
        static_cast<size_t>(manaGrid.volume()), false);
    markOccupiedInteriorProtected(manaProtected, manaGrid);
    const std::vector<Pos> corners =
        interiorCornerPositions(manaGrid);
    StateVector forcedValidSinks(
        static_cast<size_t>(manaGrid.volume()), false);
    for (const Pos& corner : corners) {
        if (manaGrid.at(
                corner.x, corner.y,
                corner.z).kind != BlockKind::Empty) {
#ifndef NDEBUG
            logDualFuelSinkCheckpoint(
                "mana_dust", -1, corner, -1, "occupied", manaGrid,
                "block=" + std::string(blockKindLabel(
                    manaGrid.at(corner.x, corner.y, corner.z).kind)));
#endif
            return std::nullopt;
        }
        manaGrid.at(corner.x, corner.y, corner.z) = {
            BlockKind::Sink, manaDustSinkType()};
        const int idx = manaGrid.index(
            corner.x, corner.y, corner.z);
        manaProtected.at(static_cast<size_t>(idx)) = true;
        forcedValidSinks.at(static_cast<size_t>(idx)) = true;
#ifndef NDEBUG
        logDualFuelSinkCheckpoint(
            "mana_dust", -1, corner, -1, "placed", manaGrid);
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
    const bool manaCornersFunctional =
        hasFunctionalSpecialManaDustCornerSinks(manaGrid, manaSim);
#ifndef NDEBUG
    logDualFuelCoolingCheckpoint(
        "afterManaDust", manaGrid, manaSim, initialDeficit,
        allowCarobbiite, allowManaDust,
        "cornersFunctional=" +
            std::to_string(manaCornersFunctional ? 1 : 0));
#endif
    if (!manaCornersFunctional) {
#ifndef NDEBUG
        logDualFuelFallbackCheckpoint(
            "manaDustFinalRejected", manaGrid, manaSim, initialDeficit,
            "cornersFunctional=" +
                std::to_string(manaCornersFunctional ? 1 : 0));
#endif
        return std::nullopt;
    }
#ifndef NDEBUG
    logDualFuelFallbackCheckpoint(
        "acceptedAfterManaDust", manaGrid, manaSim, initialDeficit);
#endif
    return manaGrid;
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
