#include "Simulator.h"

#include "Perf.h"
#include "NeutronRules.h"
#include "Rule.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace ncfr {
namespace {

constexpr int kFissionMaxSize = 24;

struct ShieldLineUse {
    int index = -1;
    double innerFlux = 0.0;
};

std::vector<Pos> allPositions(const Grid& grid) {
    std::vector<Pos> positions;
    positions.reserve(static_cast<size_t>(grid.volume()));
    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                positions.push_back({x, y, z});
            }
        }
    }
    return positions;
}

Pos indexToPos(const Grid& grid, int idx) {
    const int x = idx % grid.width();
    const int yz = idx / grid.width();
    const int y = yz % grid.height();
    const int z = yz / grid.height();
    return {x, y, z};
}

bool isFullyReflectiveReflector(const Block& block) {
    return block.kind == BlockKind::Reflector && block.type >= 0 &&
           reflectorTypes().at(static_cast<size_t>(block.type)).reflectivity >= 1.0;
}

bool sourceScanDirection(const Grid& grid, const Pos& pos, int& axis, int& direction) {
    int matchingFaces = 0;
    auto setDirection = [&](int candidateAxis, int candidateDirection) {
        axis = candidateAxis;
        direction = candidateDirection;
        ++matchingFaces;
    };

    if (pos.x == 0) {
        setDirection(0, 1);
    } else if (pos.x == grid.width() - 1) {
        setDirection(0, -1);
    }
    if (pos.y == 0) {
        setDirection(1, 1);
    } else if (pos.y == grid.height() - 1) {
        setDirection(1, -1);
    }
    if (pos.z == 0) {
        setDirection(2, 1);
    } else if (pos.z == grid.depth() - 1) {
        setDirection(2, -1);
    }

    return matchingFaces == 1;
}

void seedFromSources(const Grid& grid, StateVector& seeded) {
    for (const Pos& p : allPositions(grid)) {
        if (grid.at(p.x, p.y, p.z).kind != BlockKind::Source) {
            continue;
        }
        const int targetIndex = sourcePrimingTargetIndex(grid, p);
        if (targetIndex >= 0) {
            seeded.at(static_cast<size_t>(targetIndex)) = true;
        }
    }
}

void markCompletedLine(const std::vector<int>& moderators, const std::vector<ShieldLineUse>& shields,
                       double shieldFlux, StateVector& usedModerators, std::vector<double>& shieldFluxByIndex) {
    for (int idx : moderators) {
        usedModerators.at(static_cast<size_t>(idx)) = true;
    }
    for (const ShieldLineUse& shield : shields) {
        shieldFluxByIndex.at(static_cast<size_t>(shield.index)) += std::max(0.0, shieldFlux);
    }
}

void markCompletedFluxSinkLine(const std::vector<int>& moderators, const std::vector<ShieldLineUse>& shields,
                               StateVector& usedModerators, std::vector<double>& shieldFluxByIndex) {
    for (int idx : moderators) {
        usedModerators.at(static_cast<size_t>(idx)) = true;
    }
    for (const ShieldLineUse& shield : shields) {
        shieldFluxByIndex.at(static_cast<size_t>(shield.index)) += std::max(0.0, shield.innerFlux);
    }
}

void traceLine(const Grid& grid, const Fuel& fuel, const Pos& from, const int dir[3],
               std::vector<double>& flux, std::vector<int>& heatLinks, StateVector& usedModerators,
               StateVector& usedReflectors, std::vector<double>& shieldFluxByIndex,
               std::vector<double>& irradiatorFluxByIndex) {
    NCFR_PERF_COUNT(traceLineCalls);
    const int fromIndex = grid.index(from.x, from.y, from.z);
    Pos next{from.x + dir[0], from.y + dir[1], from.z + dir[2]};
    if (!grid.inBounds(next.x, next.y, next.z)) {
        return;
    }

    const Block& adjacent = grid.at(next.x, next.y, next.z);
    if (fuel.intrinsicFlux > 0.0) {
        if (adjacent.kind == BlockKind::FuelCell) {
            const int toIndex = grid.index(next.x, next.y, next.z);
            flux.at(static_cast<size_t>(toIndex)) += fuel.intrinsicFlux;
            ++heatLinks.at(static_cast<size_t>(fromIndex));
            ++heatLinks.at(static_cast<size_t>(toIndex));
            return;
        }
        if (adjacent.kind == BlockKind::Irradiator && adjacent.type >= 0) {
            const int toIndex = grid.index(next.x, next.y, next.z);
            irradiatorFluxByIndex.at(static_cast<size_t>(toIndex)) += fuel.intrinsicFlux;
            ++heatLinks.at(static_cast<size_t>(fromIndex));
            return;
        }
    }

    double lineFlux = fuel.intrinsicFlux;
    int moderatorCount = 0;
    std::vector<int> lineModerators;
    std::vector<ShieldLineUse> lineShields;
    Pos cur = next;
    for (int step = 1; step <= kNeutronReach; ++step) {
        if (!grid.inBounds(cur.x, cur.y, cur.z)) {
            return;
        }
        const int curIndex = grid.index(cur.x, cur.y, cur.z);
        const Block& block = grid.atIndex(curIndex);
        if (block.kind == BlockKind::Moderator && block.type >= 0) {
            const auto& moderator = moderatorTypes().at(static_cast<size_t>(block.type));
            lineFlux += moderator.fluxFactor;
            ++moderatorCount;
            lineModerators.push_back(curIndex);
        } else if (block.kind == BlockKind::Shield && block.type >= 0) {
            lineShields.push_back({curIndex, lineFlux});
        } else {
            return;
        }

        Pos target{cur.x + dir[0], cur.y + dir[1], cur.z + dir[2]};
        if (!grid.inBounds(target.x, target.y, target.z)) {
            return;
        }
        const int targetIndex = grid.index(target.x, target.y, target.z);
        const Block& targetBlock = grid.atIndex(targetIndex);
        if (targetBlock.kind == BlockKind::FuelCell) {
            flux.at(static_cast<size_t>(targetIndex)) += lineFlux;
            ++heatLinks.at(static_cast<size_t>(fromIndex));
            ++heatLinks.at(static_cast<size_t>(targetIndex));
            markCompletedLine(lineModerators, lineShields, lineFlux, usedModerators, shieldFluxByIndex);
            return;
        }
        if (targetBlock.kind == BlockKind::Irradiator && targetBlock.type >= 0) {
            irradiatorFluxByIndex.at(static_cast<size_t>(targetIndex)) += lineFlux;
            ++heatLinks.at(static_cast<size_t>(fromIndex));
            markCompletedFluxSinkLine(lineModerators, lineShields, usedModerators, shieldFluxByIndex);
            return;
        }
        if (targetBlock.kind == BlockKind::Reflector && targetBlock.type >= 0 &&
            step <= kMaxReflectorLineModerators) {
            const auto& reflector = reflectorTypes().at(static_cast<size_t>(targetBlock.type));
            usedReflectors.at(static_cast<size_t>(targetIndex)) = true;
            const double reflectedFlux = std::floor(2.0 * lineFlux * reflector.reflectivity);
            flux.at(static_cast<size_t>(fromIndex)) += reflectedFlux;
            ++heatLinks.at(static_cast<size_t>(fromIndex));
            markCompletedLine(lineModerators, lineShields, reflectedFlux, usedModerators, shieldFluxByIndex);
            return;
        }
        cur = target;
    }
}

bool isConductor(const Grid& grid, int idx, const StateVector& running, const StateVector& validSinks,
                 const StateVector& functionalShields, const StateVector& functionalIrradiators) {
    const Block& block = grid.atIndex(idx);
    switch (block.kind) {
    case BlockKind::FuelCell:
        return running.at(static_cast<size_t>(idx));
    case BlockKind::Sink:
        return validSinks.at(static_cast<size_t>(idx));
    // Plain moderators and reflectors can participate in neutron lines and placement
    // rules, but they are not NuclearCraft IFissionComponent heat conductors.
    case BlockKind::Shield:
        return functionalShields.at(static_cast<size_t>(idx));
    case BlockKind::Irradiator:
        return functionalIrradiators.at(static_cast<size_t>(idx));
    case BlockKind::Conductor:
        return true;
    default:
        return false;
    }
}

long long fuelHeating(const Fuel& fuel, int heatLinks) {
    return static_cast<long long>(std::llround(fuel.heat * std::max(1, heatLinks)));
}

long long shieldHeating(const Block& block, double flux) {
    if (block.kind != BlockKind::Shield || block.type < 0 || flux <= 0.0) {
        return 0;
    }
    const auto& shield = shieldTypes().at(static_cast<size_t>(block.type));
    return static_cast<long long>(std::floor(flux * shield.heatPerFlux));
}

long long irradiatorHeating(const Block& block, double flux) {
    if (block.kind != BlockKind::Irradiator || block.type < 0 || flux <= 0.0) {
        return 0;
    }
    const auto& recipe = irradiatorRecipeTypes().at(static_cast<size_t>(block.type));
    return static_cast<long long>(std::floor(flux * recipe.heatPerFlux));
}

bool isTrackedFunctionalBlock(const Grid& grid, int idx, const StateVector& validSinks,
                              const StateVector& functionalShields,
                              const StateVector& functionalIrradiators) {
    const Block& block = grid.atIndex(idx);
    if (block.kind == BlockKind::Sink) {
        return validSinks.at(static_cast<size_t>(idx));
    }
    if (block.kind == BlockKind::Shield) {
        return functionalShields.at(static_cast<size_t>(idx));
    }
    if (block.kind == BlockKind::Irradiator) {
        return block.type >= 0 &&
               irradiatorRecipeTypes().at(static_cast<size_t>(block.type)).heatPerFlux > 0.0 &&
               functionalIrradiators.at(static_cast<size_t>(idx));
    }
    return false;
}

} // namespace

int sourcePrimingTargetIndex(const Grid& grid, const Pos& sourcePos) {
    if (!grid.inBounds(sourcePos.x, sourcePos.y, sourcePos.z) ||
        grid.at(sourcePos.x, sourcePos.y, sourcePos.z).kind != BlockKind::Source) {
        return -1;
    }

    int axis = -1;
    int direction = 0;
    if (!sourceScanDirection(grid, sourcePos, axis, direction)) {
        return -1;
    }

    Pos pos = sourcePos;
    for (int step = 1; step <= kFissionMaxSize; ++step) {
        if (axis == 0) pos.x += direction;
        if (axis == 1) pos.y += direction;
        if (axis == 2) pos.z += direction;
        if (!grid.inBounds(pos.x, pos.y, pos.z)) {
            return -1;
        }

        const Block& block = grid.at(pos.x, pos.y, pos.z);
        if (isFullyReflectiveReflector(block)) {
            return -1;
        }
        if (block.kind == BlockKind::FuelCell) {
            return grid.index(pos.x, pos.y, pos.z);
        }
    }
    return -1;
}

template <typename FuelForIndex>
FuelSimulation simulateFuelImpl(const Grid& grid, FuelForIndex&& fuelForIndex) {
    NCFR_PERF_COUNT(simulateFuelCalls);
    NCFR_PERF_ADD(simulateFuelVolumeTotal, grid.volume());
    NCFR_PERF_SCOPE(simulateFuelNs);
    FuelSimulation result;
    const size_t volume = static_cast<size_t>(grid.volume());
    result.fluxByIndex.assign(volume, 0.0);
    result.irradiatorFluxByIndex.assign(volume, 0.0);
    result.heatLinksByIndex.assign(volume, 0);
    result.functionalCells.assign(volume, false);
    result.activeModerators.assign(volume, false);
    result.activeReflectors.assign(volume, false);
    result.functionalShields.assign(volume, false);
    result.functionalIrradiators.assign(volume, false);
    result.validSinks.assign(volume, false);
    result.heatingClusterBlocks.assign(volume, false);

    std::vector<int> cellIndices;
    for (const Pos& p : grid.interiorPositions()) {
        const int idx = grid.index(p.x, p.y, p.z);
        const Block& block = grid.atIndex(idx);
        if (block.kind == BlockKind::FuelCell && block.type >= 0) {
            cellIndices.push_back(idx);
        }
    }
    result.fuelCells = static_cast<int>(cellIndices.size());
    NCFR_PERF_ADD(simulateFuelCellTotal, result.fuelCells);
    if (cellIndices.empty()) {
        return result;
    }

    StateVector seeded(volume, false);
    for (int idx : cellIndices) {
        if (fuelForIndex(idx).selfPriming) {
            seeded.at(static_cast<size_t>(idx)) = true;
        }
    }
    seedFromSources(grid, seeded);

    StateVector running = seeded;
    std::vector<double> shieldFluxByIndex(volume, 0.0);
    std::vector<double> irradiatorFluxByIndex(volume, 0.0);
    static constexpr int dirs[6][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };

    for (int iteration = 0; iteration < 16; ++iteration) {
        NCFR_PERF_COUNT(simulateFuelIterations);
        std::fill(result.fluxByIndex.begin(), result.fluxByIndex.end(), 0.0);
        std::fill(result.heatLinksByIndex.begin(), result.heatLinksByIndex.end(), 0);
        std::fill(result.activeModerators.begin(), result.activeModerators.end(), false);
        std::fill(result.activeReflectors.begin(), result.activeReflectors.end(), false);
        std::fill(shieldFluxByIndex.begin(), shieldFluxByIndex.end(), 0.0);
        std::fill(irradiatorFluxByIndex.begin(), irradiatorFluxByIndex.end(), 0.0);

        for (int idx : cellIndices) {
            if (!running.at(static_cast<size_t>(idx))) {
                continue;
            }
            const Pos from = indexToPos(grid, idx);
            const Fuel& fuel = fuelForIndex(idx);
            for (const auto& dir : dirs) {
                traceLine(grid, fuel, from, dir, result.fluxByIndex, result.heatLinksByIndex,
                          result.activeModerators, result.activeReflectors, shieldFluxByIndex,
                          irradiatorFluxByIndex);
            }
        }

        StateVector nextRunning = seeded;
        for (int idx : cellIndices) {
            if (result.fluxByIndex.at(static_cast<size_t>(idx)) + 1e-9 >= fuelForIndex(idx).criticality) {
                nextRunning.at(static_cast<size_t>(idx)) = true;
            }
        }
        if (nextRunning == running) {
            break;
        }
        running.swap(nextRunning);
    }

    for (int idx : cellIndices) {
        if (running.at(static_cast<size_t>(idx)) &&
            result.fluxByIndex.at(static_cast<size_t>(idx)) + 1e-9 >= fuelForIndex(idx).criticality) {
            result.functionalCells.at(static_cast<size_t>(idx)) = true;
            ++result.runningCells;
        }
    }
    for (int idx = 0; idx < grid.volume(); ++idx) {
        result.irradiatorFluxByIndex.at(static_cast<size_t>(idx)) = irradiatorFluxByIndex.at(static_cast<size_t>(idx));
        result.functionalShields.at(static_cast<size_t>(idx)) =
            grid.atIndex(idx).kind == BlockKind::Shield && shieldFluxByIndex.at(static_cast<size_t>(idx)) > 0.0;
        result.functionalIrradiators.at(static_cast<size_t>(idx)) =
            grid.atIndex(idx).kind == BlockKind::Irradiator && irradiatorFluxByIndex.at(static_cast<size_t>(idx)) > 0.0;
    }

    RuleContext context;
    context.functionalCells = &result.functionalCells;
    context.activeModerators = &result.activeModerators;
    context.activeReflectors = &result.activeReflectors;
    context.functionalShields = &result.functionalShields;
    context.functionalIrradiators = &result.functionalIrradiators;
    result.validSinks = evaluateValidSinks(grid, context);

    for (int idx = 0; idx < grid.volume(); ++idx) {
        const Block& block = grid.atIndex(idx);
        if (block.kind == BlockKind::FuelCell && result.functionalCells.at(static_cast<size_t>(idx))) {
            result.rawHeating += fuelHeating(fuelForIndex(idx), result.heatLinksByIndex.at(static_cast<size_t>(idx)));
        } else if (block.kind == BlockKind::Shield && result.functionalShields.at(static_cast<size_t>(idx))) {
            result.rawHeating += shieldHeating(block, shieldFluxByIndex.at(static_cast<size_t>(idx)));
        } else if (block.kind == BlockKind::Irradiator && result.functionalIrradiators.at(static_cast<size_t>(idx))) {
            result.rawHeating += irradiatorHeating(block, irradiatorFluxByIndex.at(static_cast<size_t>(idx)));
        } else if (block.kind == BlockKind::Sink && result.validSinks.at(static_cast<size_t>(idx)) && block.type >= 0) {
            result.cooling += sinkTypes().at(static_cast<size_t>(block.type)).cooling;
        }
    }

    StateVector visited(volume, false);
    result.minClusterMargin = std::numeric_limits<long long>::max();
    for (int start = 0; start < grid.volume(); ++start) {
        if (visited.at(static_cast<size_t>(start)) ||
            !isConductor(grid, start, running, result.validSinks, result.functionalShields,
                         result.functionalIrradiators)) {
            continue;
        }
        ClusterStats cluster;
        std::vector<int> clusterIndices;
        std::queue<int> queue;
        queue.push(start);
        visited.at(static_cast<size_t>(start)) = true;
        while (!queue.empty()) {
            const int idx = queue.front();
            queue.pop();
            clusterIndices.push_back(idx);
            ++cluster.components;
            const Pos p = indexToPos(grid, idx);
            const Block& block = grid.atIndex(idx);
            if (block.kind == BlockKind::FuelCell && result.functionalCells.at(static_cast<size_t>(idx))) {
                cluster.rawHeating += fuelHeating(fuelForIndex(idx), result.heatLinksByIndex.at(static_cast<size_t>(idx)));
            } else if (block.kind == BlockKind::Shield && result.functionalShields.at(static_cast<size_t>(idx))) {
                cluster.rawHeating += shieldHeating(block, shieldFluxByIndex.at(static_cast<size_t>(idx)));
            } else if (block.kind == BlockKind::Irradiator && result.functionalIrradiators.at(static_cast<size_t>(idx))) {
                cluster.rawHeating += irradiatorHeating(block, irradiatorFluxByIndex.at(static_cast<size_t>(idx)));
            } else if (block.kind == BlockKind::Sink && block.type >= 0 && result.validSinks.at(static_cast<size_t>(idx))) {
                cluster.cooling += sinkTypes().at(static_cast<size_t>(block.type)).cooling;
            }

            grid.forEachNeighbor6(p, [&](const Pos& n) {
                if (grid.isBoundary(n.x, n.y, n.z) && isCasingLike(grid.at(n.x, n.y, n.z).kind)) {
                    cluster.connectedToWall = true;
                }
                const int nIdx = grid.index(n.x, n.y, n.z);
                if (!visited.at(static_cast<size_t>(nIdx)) &&
                    isConductor(grid, nIdx, running, result.validSinks, result.functionalShields,
                                result.functionalIrradiators)) {
                    visited.at(static_cast<size_t>(nIdx)) = true;
                    queue.push(nIdx);
                }
            });
        }
        if (cluster.rawHeating > 0) {
            result.minClusterMargin = std::min(result.minClusterMargin, cluster.cooling - cluster.rawHeating);
            for (int idx : clusterIndices) {
                result.heatingClusterBlocks.at(static_cast<size_t>(idx)) = true;
            }
        }
        result.clusters.push_back(cluster);
    }

    if (result.minClusterMargin == std::numeric_limits<long long>::max()) {
        result.minClusterMargin = 0;
    }

    for (int idx = 0; idx < grid.volume(); ++idx) {
        if (isTrackedFunctionalBlock(grid, idx, result.validSinks, result.functionalShields, result.functionalIrradiators) &&
            !result.heatingClusterBlocks.at(static_cast<size_t>(idx))) {
            ++result.disconnectedFunctionalBlocks;
        }
    }

    result.compatible = result.runningCells == result.fuelCells && result.fuelCells > 0;
    if (result.compatible) {
        for (const ClusterStats& cluster : result.clusters) {
            if (cluster.rawHeating > 0 && (!cluster.connectedToWall || cluster.cooling < cluster.rawHeating)) {
                result.compatible = false;
                break;
            }
        }
    }
    return result;
}

FuelSimulation simulateFuel(const Grid& grid, const Fuel& fuel) {
    return simulateFuelImpl(grid, [&](int) -> const Fuel& { return fuel; });
}

FuelSimulation simulateMixedFuel(const Grid& grid) {
    return simulateFuelImpl(grid, [&](int idx) -> const Fuel& {
        const Block& block = grid.atIndex(idx);
        return fuels().at(static_cast<size_t>(block.type));
    });
}

bool hasSafeFuelFlux(const Grid& grid, const FuelSimulation& sim) {
    if (sim.fluxByIndex.size() < static_cast<size_t>(grid.volume()) ||
        sim.functionalCells.size() < static_cast<size_t>(grid.volume())) {
        return false;
    }

    for (const Pos& pos : grid.interiorPositions()) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        const Block& block = grid.atIndex(idx);
        if (block.kind != BlockKind::FuelCell || block.type < 0 ||
            block.type >= static_cast<int>(fuels().size())) {
            continue;
        }
        if (!sim.functionalCells.at(static_cast<size_t>(idx))) {
            return false;
        }
        const Fuel& fuel = fuels().at(static_cast<size_t>(block.type));
        if (sim.fluxByIndex.at(static_cast<size_t>(idx)) > 2.0 * fuel.criticality + 1e-9) {
            return false;
        }
    }
    return true;
}

bool isSafeOperatingSimulation(const Grid& grid, const FuelSimulation& sim) {
    return sim.compatible && sim.minClusterMargin >= 0 && sim.disconnectedFunctionalBlocks == 0 &&
           hasSafeFuelFlux(grid, sim);
}

} // namespace ncfr
