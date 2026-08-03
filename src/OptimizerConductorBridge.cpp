#include "OptimizerConductorBridge.h"

#include "OptimizerCommon.h"
#include "OptimizerDiagnostics.h"
#include "Perf.h"

#include <algorithm>
#include <deque>
#include <functional>
#include <optional>
#include <sstream>
#include <vector>
namespace ncfr::optimizer_detail {

struct HeatingClusterInfo {
    std::vector<Pos> blocks;
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

} // namespace ncfr::optimizer_detail
