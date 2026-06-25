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
constexpr long long kSpecialManaDustCoolingDeficit = 640;

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
    int targetLineCount = 1;
};

bool isSpecialManaDustRequest(const BuildRequest& request) {
    return request.fuelIndices.size() == 1 &&
           usesSpecialManaDustCornerSinks(fuels().at(static_cast<size_t>(request.fuelIndices.front())));
}

bool hasSpecialManaDustCoolingDeficit(const FuelSimulation& sim) {
    const long long deficit = sim.rawHeating - sim.cooling;
    return deficit > 0 && deficit <= kSpecialManaDustCoolingDeficit;
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

std::vector<Dimension> singleFuelSearchDimensions(const FuelActivationProfile& profile) {
    static constexpr std::array<Dimension, 4> dims = {{
        {5, 5, 5},
        {15, 15, 15},
        {18, 18, 18},
        {24, 24, 24},
    }};

    size_t start = 0;
    if (profile.minSearchInteriorSize >= 18) {
        start = 2;
    } else if (profile.minSearchInteriorSize >= 15) {
        start = 1;
    }

    return std::vector<Dimension>(dims.begin() + static_cast<std::ptrdiff_t>(start), dims.end());
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

void markSourcePathsProtected(StateVector& protectedPositions, const Grid& grid, const Pos& fuelPos,
                              const std::vector<int>& sourceDirections) {
    for (int sourceDirection : sourceDirections) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(sourceDirection));
        Pos pos = sourcePositionForDirection(grid, fuelPos, dir);
        while (grid.inBounds(pos.x, pos.y, pos.z)) {
            markProtected(protectedPositions, grid, pos);
            pos.x -= dir.dx;
            pos.y -= dir.dy;
            pos.z -= dir.dz;
            if (!grid.inBounds(pos.x, pos.y, pos.z) ||
                (pos.x == fuelPos.x && pos.y == fuelPos.y && pos.z == fuelPos.z)) {
                break;
            }
        }
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
    markSourcePathsProtected(protectedPositions, grid, fuelPos, sourceDirections);
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
    bool connectedToWall = false;
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
           (!sim.compatible || sim.disconnectedFunctionalBlocks != 0);
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
                if (grid.isBoundary(n.x, n.y, n.z) && isCasingLike(grid.at(n.x, n.y, n.z).kind)) {
                    cluster.connectedToWall = true;
                    return;
                }
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

bool isInteriorTouchingWall(const Grid& grid, const Pos& pos) {
    bool touchesWall = false;
    grid.forEachNeighbor6(pos, [&](const Pos& n) {
        if (grid.isBoundary(n.x, n.y, n.z) && isCasingLike(grid.at(n.x, n.y, n.z).kind)) {
            touchesWall = true;
        }
    });
    return touchesWall;
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

    int mainCluster = 0;
    for (int i = 0; i < static_cast<int>(clusters.size()); ++i) {
        if (clusters.at(static_cast<size_t>(i)).connectedToWall) {
            mainCluster = i;
            break;
        }
    }

    std::vector<int> connectedClusters{mainCluster};
    if (!clusters.at(static_cast<size_t>(mainCluster)).connectedToWall) {
        StateVector targetMask(static_cast<size_t>(grid.volume()), false);
        const auto wallPath = shortestConductorPath(
            grid, clusters.at(static_cast<size_t>(mainCluster)).blocks, targetMask, protectedPositions,
            [&](const Pos& pos) { return isInteriorTouchingWall(grid, pos); }, cancelRequested);
        if (!wallPath.has_value()) {
            result.reason = "wallPathMissing";
            return result;
        }
        Grid trial = grid;
        const int added = placeConductorsOnPath(trial, *wallPath, targetMask, initialSim.heatingClusterBlocks);
        FuelSimulation trialSim = simulateMixedFuel(trial);
        if (!bridgeStillSafe(trial, trialSim)) {
            result.reason = "wallPathUnsafe";
            return result;
        }
        result.conductorsAdded += added;
        grid = std::move(trial);
        result.sim = std::move(trialSim);
    }

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
    result.success = isAccepted(result.grid, result.sim);
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
        &candidate->protectedPositions, search.cancelRequested);
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
            if (std::optional<OptimizationResult> result =
                    enumerateSingleFuelSkeletonSpecs(search, direction + 1, current)) {
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
        if (std::optional<OptimizationResult> result =
                enumerateSingleFuelSkeletonSpecs(localSearch, 0, current)) {
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

std::optional<Grid> compactInteriorPlanesPreservingSources(const Grid& grid, const BuildRequest& request,
                                                           const std::vector<int>& sourceDirections,
                                                           const std::vector<FuelLineSpec>& fuelLines,
                                                           int paddingPlanes = 0,
                                                           bool keepConductors = false) {
    NCFR_PERF_COUNT(compactInteriorPlanesCalls);
    NCFR_PERF_SCOPE(compactInteriorPlanesNs);
    std::vector<bool> keepX(static_cast<size_t>(grid.internalA() + 1), false);
    std::vector<bool> keepY(static_cast<size_t>(grid.internalB() + 1), false);
    std::vector<bool> keepZ(static_cast<size_t>(grid.internalC() + 1), false);

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
        keepRange(keepX, pos.x, grid.internalA());
        keepRange(keepY, pos.y, grid.internalB());
        keepRange(keepZ, pos.z, grid.internalC());
    }

    const int newA = static_cast<int>(std::count(keepX.begin(), keepX.end(), true));
    const int newB = static_cast<int>(std::count(keepY.begin(), keepY.end(), true));
    const int newC = static_cast<int>(std::count(keepZ.begin(), keepZ.end(), true));
    if (newA <= 0 || newB <= 0 || newC <= 0) {
        return std::nullopt;
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
        if (!keepConductors && block.kind == BlockKind::Conductor &&
            (!keepX.at(static_cast<size_t>(pos.x)) ||
             !keepY.at(static_cast<size_t>(pos.y)) ||
             !keepZ.at(static_cast<size_t>(pos.z)))) {
            continue;
        }
        compacted.at(mapX.at(static_cast<size_t>(pos.x)),
                     mapY.at(static_cast<size_t>(pos.y)),
                     mapZ.at(static_cast<size_t>(pos.z))) = block;
    }

    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(compacted);
    if (fuelPositions.size() != request.fuelIndices.size() ||
        !restoreDirectionalFuelLines(compacted, request, sourceDirections, fuelLines)) {
        return std::nullopt;
    }
    return compacted;
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
    if (!isAccepted(*finalCompacted, finalSim)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("finalNotAccepted", *finalCompacted, &finalSim, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, classifyFinalizationFailure(*finalCompacted, finalSim, request)};
    }
#ifndef NDEBUG
    const std::string detail = directionalGridDetail("accepted", *finalCompacted, &finalSim, request,
                                                     sourceDirections, reflectorDirections);
    logFinalizeCheckpoint("finalize.accept", detail, 0, kDefaultImproveOptions);
#endif
    addFuelCellPorts(*finalCompacted, request);
    OptimizationResult result(std::move(*finalCompacted), request);
    result.minCoolingMargin = finalSim.minClusterMargin;
    result.usefulBlocks = countUsefulBlocks(result.grid);
    result.disconnectedFunctionalBlocks = finalSim.disconnectedFunctionalBlocks;
    result.functionalIrradiators = countFunctionalIrradiators(finalSim);
    result.irradiatorFlux = 0.0;
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

bool hasFunctionalSpecialManaDustCornerSinks(const Grid& grid, const FuelSimulation& sim) {
    if (grid.internalA() < 2 || grid.internalB() < 2 || grid.internalC() < 2 ||
        sim.validSinks.size() < static_cast<size_t>(grid.volume()) ||
        sim.heatingClusterBlocks.size() < static_cast<size_t>(grid.volume())) {
        return false;
    }
    int functionalManaDustCorners = 0;
    for (const Pos& corner : interiorCornerPositions(grid)) {
        if (!isManaDustSink(grid.at(corner.x, corner.y, corner.z))) {
            continue;
        }
        const int idx = grid.index(corner.x, corner.y, corner.z);
        if (!sim.validSinks.at(static_cast<size_t>(idx)) ||
            !sim.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            return false;
        }
        ++functionalManaDustCorners;
    }
    return functionalManaDustCorners > 0;
}

std::vector<Pos> placeSpecialManaDustCornerSinks(Grid& grid, StateVector& protectedPositions,
                                                 const StateVector* baseProtectedPositions) {
    if (manaDustSinkType() < 0 ||
        grid.internalA() < 2 || grid.internalB() < 2 || grid.internalC() < 2) {
        return {};
    }
    const size_t volume = static_cast<size_t>(grid.volume());
    if (baseProtectedPositions != nullptr && baseProtectedPositions->size() != volume) {
        return {};
    }
    protectedPositions = baseProtectedPositions != nullptr
        ? *baseProtectedPositions
        : StateVector(volume, false);

    const std::vector<Pos> corners = interiorCornerPositions(grid);
    std::vector<Pos> placedCorners;
    for (const Pos& corner : corners) {
        const int idx = grid.index(corner.x, corner.y, corner.z);
        if (protectedPositions.at(static_cast<size_t>(idx))) {
            continue;
        }
        const Block& block = grid.atIndex(idx);
        if (!isManaDustSink(block) && block.kind != BlockKind::Empty && !isSupportMutable(block)) {
            continue;
        }
        grid.at(corner.x, corner.y, corner.z) = {BlockKind::Sink, manaDustSinkType()};
        markProtected(protectedPositions, grid, corner);
        placedCorners.push_back(corner);
    }
    return placedCorners;
}

bool anyHeatingClusterBlock(const FuelSimulation& sim) {
    return std::find(sim.heatingClusterBlocks.begin(), sim.heatingClusterBlocks.end(), 1U) !=
           sim.heatingClusterBlocks.end();
}

std::optional<FinalizeResult> trySpecialManaDustFinalization(
    const Grid& grid, const FuelSimulation& sim, const BuildRequest& request,
    const std::vector<int>& sourceDirections, const std::vector<FuelLineSpec>& fuelLines,
    const StateVector* protectedPositions, const std::atomic_bool* cancelRequested) {
    (void)protectedPositions;
    if (!isSpecialManaDustRequest(request) ||
        !isPreCompactRunnable(sim) ||
        !hasSafeFuelFlux(grid, sim) ||
        !hasSpecialManaDustCoolingDeficit(sim)) {
        return std::nullopt;
    }

    std::optional<Grid> compactedBase =
        compactInteriorPlanesPreservingSources(grid, request, sourceDirections, fuelLines);
    if (!compactedBase.has_value()) {
        return std::nullopt;
    }
    Grid specialGrid = std::move(*compactedBase);
    FuelSimulation baseSim = simulateMixedFuel(specialGrid);
    if (!isPreCompactRunnable(baseSim) ||
        !hasSafeFuelFlux(specialGrid, baseSim) ||
        !hasSpecialManaDustCoolingDeficit(baseSim)) {
        return std::nullopt;
    }

    StateVector specialProtected(static_cast<size_t>(specialGrid.volume()), false);
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(specialGrid);
    if (fuelPositions.size() != request.fuelIndices.size()) {
        return std::nullopt;
    }
    const Pos fuelPos = fuelPositions.front();
    markProtected(specialProtected, specialGrid, fuelPos);
    markSourcePathsProtected(specialProtected, specialGrid, fuelPos, sourceDirections);
    for (const FuelLineSpec& line : fuelLines) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(line.direction));
        for (int distance = 1; distance <= line.moderatorCount + 1; ++distance) {
            markProtected(specialProtected, specialGrid, offset(fuelPos, dir, distance));
        }
    }

    std::vector<Pos> manaDustCorners =
        placeSpecialManaDustCornerSinks(specialGrid, specialProtected, &specialProtected);
    if (manaDustCorners.empty()) {
        return std::nullopt;
    }

    FuelSimulation currentSim = simulateMixedFuel(specialGrid);
    if (!isPreCompactRunnable(currentSim) ||
        !hasSafeFuelFlux(specialGrid, currentSim) ||
        !anyHeatingClusterBlock(currentSim)) {
        return std::nullopt;
    }

    const std::vector<Pos>& corners = manaDustCorners;
    for (const Pos& corner : corners) {
        throwIfCancelled(cancelRequested);
        const int cornerIdx = specialGrid.index(corner.x, corner.y, corner.z);
        if (!currentSim.validSinks.at(static_cast<size_t>(cornerIdx))) {
            return std::nullopt;
        }
        if (currentSim.heatingClusterBlocks.at(static_cast<size_t>(cornerIdx))) {
            continue;
        }

        const StateVector targetMask = currentSim.heatingClusterBlocks;
        const auto path = shortestConductorPath(
            specialGrid, {corner}, targetMask, &specialProtected,
            [&](const Pos& pos) {
                return targetMask.at(static_cast<size_t>(
                    specialGrid.index(pos.x, pos.y, pos.z)));
            },
            cancelRequested);
        if (!path.has_value()) {
            return std::nullopt;
        }

        Grid trial = specialGrid;
        placeConductorsOnPath(trial, *path, targetMask, specialProtected);
        FuelSimulation trialSim = simulateMixedFuel(trial);
        if (!isPreCompactRunnable(trialSim) ||
            !hasSafeFuelFlux(trial, trialSim) ||
            !trialSim.validSinks.at(static_cast<size_t>(cornerIdx))) {
            return std::nullopt;
        }
        specialGrid = std::move(trial);
        currentSim = std::move(trialSim);
    }

    pruneInactiveSupport(specialGrid, &specialProtected);
    currentSim = simulateMixedFuel(specialGrid);
    if (!isAccepted(specialGrid, currentSim)) {
        return std::nullopt;
    }

    FinalizeResult finalResult = acceptedResultFromImprovedGrid(
        std::move(specialGrid), currentSim, request, sourceDirections, fuelLines,
        "specialManaDustCompactValidationFailed", true);
    if (!finalResult.result.has_value()) {
        return std::nullopt;
    }
    const FuelSimulation finalSim = simulateMixedFuel(finalResult.result->grid);
    if (!isAccepted(finalResult.result->grid, finalSim) ||
        !hasFunctionalSpecialManaDustCornerSinks(finalResult.result->grid, finalSim)) {
        return std::nullopt;
    }
    return finalResult;
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
    if (!isAccepted(bridge.grid, bridge.sim)) {
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
                                               const std::atomic_bool* cancelRequested) {
    const std::vector<int> reflectorDirections = lineDirections(fuelLines);
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
    if (!isPreCompactRunnable(sim)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("preOptimizeNotRunnable", grid, &sim, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, classifyFinalizationFailure(grid, sim, request)};
    }

    if (isAccepted(grid, sim)) {
        FinalizeResult finalResult = acceptedResultFromImprovedGrid(
            grid, sim, request, sourceDirections, fuelLines, "finalCompactValidationFailed");
        if (finalResult.result.has_value()) {
            return finalResult;
        }
    }
    if (std::optional<FinalizeResult> specialResult =
            trySpecialManaDustFinalization(grid, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
        return std::move(*specialResult);
    }
    if (std::optional<FinalizeResult> bridgeResult =
            tryConductorBridgeFinalization(grid, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
        return std::move(*bridgeResult);
    }

    fillSupportBlocks(grid, &supportOptions, protectedPositions);
    Grid filledBridgeBase = grid;
    pruneInactiveSupport(filledBridgeBase, protectedPositions);
    sim = simulateMixedFuel(filledBridgeBase);
    if (isAccepted(filledBridgeBase, sim)) {
        FinalizeResult finalResult = acceptedResultFromImprovedGrid(
            std::move(filledBridgeBase), sim, request, sourceDirections, fuelLines,
            "finalCompactValidationFailed");
        if (finalResult.result.has_value()) {
            return finalResult;
        }
    }
    if (std::optional<FinalizeResult> specialResult =
            trySpecialManaDustFinalization(filledBridgeBase, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
        return std::move(*specialResult);
    }
    if (std::optional<FinalizeResult> bridgeResult =
            tryConductorBridgeFinalization(filledBridgeBase, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
        return std::move(*bridgeResult);
    }

    Grid improved = improveSupportBlocks(std::move(grid), cancelRequested, kDefaultImproveOptions, &supportOptions,
                                         protectedPositions, true);
    pruneInactiveSupport(improved, protectedPositions);
    sim = simulateMixedFuel(improved);
    if (std::optional<FinalizeResult> specialResult =
            trySpecialManaDustFinalization(improved, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
        return std::move(*specialResult);
    }
    if (std::optional<FinalizeResult> bridgeResult =
            tryConductorBridgeFinalization(improved, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
        return std::move(*bridgeResult);
    }
    if (classifyFinalizationFailure(improved, sim, request) == FinalizeFailureKind::CoolingDeficit) {
        const Grid protectedBaseline = improved;
        improved = expandCoolingWithPreserver(
            std::move(improved),
            [protectedPositions, protectedBaseline](Grid& candidate) {
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
    }
    if (std::optional<FinalizeResult> specialResult =
            trySpecialManaDustFinalization(improved, sim, request, sourceDirections, fuelLines,
                                           protectedPositions, cancelRequested)) {
        return std::move(*specialResult);
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
    const FuelActivationProfile& profile = fuelActivationProfile(request.fuelIndices.front());
    const std::vector<Dimension> dims = singleFuelSearchDimensions(profile);

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
