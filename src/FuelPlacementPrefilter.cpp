#include "FuelPlacementPrefilter.h"

#include "Data.h"
#include "Optimizer.h"
#include "Simulator.h"
#include "StateVector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ncfr {
namespace {

constexpr int kNeutronReach = 4;
constexpr double kFluxEpsilon = 1e-9;

struct Direction {
    int dx = 0;
    int dy = 0;
    int dz = 0;
};

constexpr std::array<Direction, 6> kDirections = {{
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1},
}};

Pos indexToPos(const Grid& grid, int idx) {
    const int x = idx % grid.width();
    const int yz = idx / grid.width();
    const int y = yz % grid.height();
    const int z = yz / grid.height();
    return {x, y, z};
}

int minLineCount(double requiredFlux, double fluxPerLine) {
    if (requiredFlux <= 0.0) {
        return 0;
    }
    if (fluxPerLine <= 0.0) {
        return std::numeric_limits<int>::max();
    }
    return static_cast<int>(std::ceil((requiredFlux - kFluxEpsilon) / fluxPerLine));
}

int minSearchInteriorSize(double heat) {
    if (heat > 900.0) {
        return 18;
    }
    if (heat > 100.0) {
        return 15;
    }
    return 5;
}

std::vector<int> fuelCellIndices(const Grid& grid) {
    std::vector<int> indices;
    for (const Pos& pos : grid.interiorPositions()) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        const Block& block = grid.atIndex(idx);
        if (block.kind == BlockKind::FuelCell && block.type >= 0) {
            indices.push_back(idx);
        }
    }
    return indices;
}

bool hasValidFuelType(const Grid& grid, const std::vector<int>& cellIndices) {
    for (int idx : cellIndices) {
        const int type = grid.atIndex(idx).type;
        if (type < 0 || type >= static_cast<int>(fuels().size())) {
            return false;
        }
    }
    return true;
}

void seedRunningFuelCells(const Grid& grid, const std::vector<int>& cellIndices, StateVector& seeded) {
    for (int idx : cellIndices) {
        const Fuel& fuel = fuels().at(static_cast<size_t>(grid.atIndex(idx).type));
        if (fuel.selfPriming) {
            seeded.at(static_cast<size_t>(idx)) = true;
        }
    }
    for (int idx = 0; idx < grid.volume(); ++idx) {
        if (grid.atIndex(idx).kind != BlockKind::Source) {
            continue;
        }
        const Pos sourcePos = indexToPos(grid, idx);
        const int target = sourcePrimingTargetIndex(grid, sourcePos);
        if (target >= 0 && grid.atIndex(target).kind == BlockKind::FuelCell) {
            seeded.at(static_cast<size_t>(target)) = true;
        }
    }
}

bool anySeeded(const std::vector<int>& cellIndices, const StateVector& seeded) {
    return std::any_of(cellIndices.begin(), cellIndices.end(), [&](int idx) {
        return seeded.at(static_cast<size_t>(idx));
    });
}

void traceFuelRelationLine(const Grid& grid, const Fuel& fuel, const Pos& from, const Direction& dir,
                           std::vector<double>& fluxByIndex, std::vector<double>& irradiatorFluxByIndex) {
    const int fromIndex = grid.index(from.x, from.y, from.z);
    Pos next{from.x + dir.dx, from.y + dir.dy, from.z + dir.dz};
    if (!grid.inBounds(next.x, next.y, next.z)) {
        return;
    }

    const Block& adjacent = grid.at(next.x, next.y, next.z);
    if (fuel.intrinsicFlux > 0.0) {
        if (adjacent.kind == BlockKind::FuelCell) {
            const int toIndex = grid.index(next.x, next.y, next.z);
            fluxByIndex.at(static_cast<size_t>(toIndex)) += fuel.intrinsicFlux;
            return;
        }
        if (adjacent.kind == BlockKind::Irradiator && adjacent.type >= 0) {
            const int toIndex = grid.index(next.x, next.y, next.z);
            irradiatorFluxByIndex.at(static_cast<size_t>(toIndex)) += fuel.intrinsicFlux;
            return;
        }
    }

    double lineFlux = fuel.intrinsicFlux;
    int moderatorCount = 0;
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
        } else if (block.kind == BlockKind::Shield && block.type >= 0) {
            // Shields preserve the neutron line but do not add relation flux.
        } else {
            return;
        }

        Pos target{cur.x + dir.dx, cur.y + dir.dy, cur.z + dir.dz};
        if (!grid.inBounds(target.x, target.y, target.z)) {
            return;
        }
        const int targetIndex = grid.index(target.x, target.y, target.z);
        const Block& targetBlock = grid.atIndex(targetIndex);
        if (targetBlock.kind == BlockKind::FuelCell) {
            fluxByIndex.at(static_cast<size_t>(targetIndex)) += lineFlux;
            return;
        }
        if (targetBlock.kind == BlockKind::Irradiator && targetBlock.type >= 0) {
            irradiatorFluxByIndex.at(static_cast<size_t>(targetIndex)) += lineFlux;
            return;
        }
        if (targetBlock.kind == BlockKind::Reflector && targetBlock.type >= 0 &&
            moderatorCount <= kNeutronReach / 2) {
            const auto& reflector = reflectorTypes().at(static_cast<size_t>(targetBlock.type));
            fluxByIndex.at(static_cast<size_t>(fromIndex)) += std::floor(2.0 * lineFlux * reflector.reflectivity);
            return;
        }
        cur = target;
    }
}

void traceFuelRelations(const Grid& grid, const std::vector<int>& cellIndices, const StateVector& running,
                        std::vector<double>& fluxByIndex, std::vector<double>& irradiatorFluxByIndex) {
    std::fill(fluxByIndex.begin(), fluxByIndex.end(), 0.0);
    std::fill(irradiatorFluxByIndex.begin(), irradiatorFluxByIndex.end(), 0.0);
    for (int idx : cellIndices) {
        if (!running.at(static_cast<size_t>(idx))) {
            continue;
        }
        const Fuel& fuel = fuels().at(static_cast<size_t>(grid.atIndex(idx).type));
        const Pos from = indexToPos(grid, idx);
        for (const Direction& dir : kDirections) {
            traceFuelRelationLine(grid, fuel, from, dir, fluxByIndex, irradiatorFluxByIndex);
        }
    }
}

FuelRelationPrefilterResult resultWithReason(FuelRelationPrefilterResult result,
                                             FuelRelationRejectReason reason) {
    result.accepted = reason == FuelRelationRejectReason::None;
    result.reason = reason;
    return result;
}

} // namespace

const std::vector<FuelActivationProfile>& fuelActivationProfiles() {
    static const std::vector<FuelActivationProfile> profiles = [] {
        std::vector<FuelActivationProfile> result;
        result.reserve(fuels().size());
        const double heavyWaterFlux = moderatorTypes().at(2).fluxFactor;
        const double fullReflectorFlux =
            std::floor(2.0 * heavyWaterFlux * reflectorTypes().at(0).reflectivity);
        const double halfReflectorFlux =
            std::floor(2.0 * heavyWaterFlux * reflectorTypes().at(1).reflectivity);
        for (int index = 0; index < static_cast<int>(fuels().size()); ++index) {
            const Fuel& fuel = fuels().at(static_cast<size_t>(index));
            result.push_back({index,
                              fuel.criticality,
                              fuel.intrinsicFlux,
                              fuel.heat,
                              fuel.selfPriming,
                              minLineCount(fuel.criticality, fullReflectorFlux),
                              minLineCount(fuel.criticality, halfReflectorFlux),
                              minLineCount(fuel.criticality, heavyWaterFlux),
                              minSearchInteriorSize(fuel.heat)});
        }
        return result;
    }();
    return profiles;
}

const FuelActivationProfile& fuelActivationProfile(int fuelIndex) {
    if (fuelIndex < 0 || fuelIndex >= static_cast<int>(fuelActivationProfiles().size())) {
        throw std::out_of_range("fuel activation profile index out of range");
    }
    return fuelActivationProfiles().at(static_cast<size_t>(fuelIndex));
}

FuelRelationPrefilterResult prefilterFuelRelations(const Grid& grid, const BuildRequest& request) {
    FuelRelationPrefilterResult result;
    const std::vector<int> cellIndices = fuelCellIndices(grid);
    result.fuelCells = static_cast<int>(cellIndices.size());
    if (cellIndices.empty()) {
        return resultWithReason(result, FuelRelationRejectReason::NoFuelCells);
    }
    if (!hasValidFuelType(grid, cellIndices)) {
        return resultWithReason(result, FuelRelationRejectReason::MissingFuelData);
    }

    const size_t volume = static_cast<size_t>(grid.volume());
    StateVector seeded(volume, false);
    seedRunningFuelCells(grid, cellIndices, seeded);
    if (!anySeeded(cellIndices, seeded)) {
        return resultWithReason(result, FuelRelationRejectReason::NoSeed);
    }

    StateVector running = seeded;
    std::vector<double> fluxByIndex(volume, 0.0);
    std::vector<double> irradiatorFluxByIndex(volume, 0.0);
    for (int iteration = 0; iteration < 16; ++iteration) {
        traceFuelRelations(grid, cellIndices, running, fluxByIndex, irradiatorFluxByIndex);
        StateVector nextRunning = seeded;
        for (int idx : cellIndices) {
            const Fuel& fuel = fuels().at(static_cast<size_t>(grid.atIndex(idx).type));
            if (fluxByIndex.at(static_cast<size_t>(idx)) + kFluxEpsilon >= fuel.criticality) {
                nextRunning.at(static_cast<size_t>(idx)) = true;
            }
        }
        if (nextRunning == running) {
            break;
        }
        running.swap(nextRunning);
    }

    result.weakestFuelMargin = std::numeric_limits<double>::max();
    for (int idx : cellIndices) {
        const Fuel& fuel = fuels().at(static_cast<size_t>(grid.atIndex(idx).type));
        const double margin = fluxByIndex.at(static_cast<size_t>(idx)) - fuel.criticality;
        result.weakestFuelMargin = std::min(result.weakestFuelMargin, margin);
        if (running.at(static_cast<size_t>(idx)) && margin + kFluxEpsilon >= 0.0) {
            ++result.runningCells;
        }
    }
    if (result.weakestFuelMargin == std::numeric_limits<double>::max()) {
        result.weakestFuelMargin = 0.0;
    }
    if (result.runningCells != result.fuelCells) {
        return resultWithReason(result, FuelRelationRejectReason::FuelNotRunnable);
    }

    for (int idx = 0; idx < grid.volume(); ++idx) {
        const Block& block = grid.atIndex(idx);
        if (block.kind == BlockKind::Irradiator && irradiatorFluxByIndex.at(static_cast<size_t>(idx)) > 0.0) {
            ++result.functionalIrradiators;
        }
    }
    (void)request;

    return resultWithReason(result, FuelRelationRejectReason::None);
}

const char* fuelRelationRejectReasonName(FuelRelationRejectReason reason) {
    switch (reason) {
    case FuelRelationRejectReason::None:
        return "none";
    case FuelRelationRejectReason::NoFuelCells:
        return "noFuelCells";
    case FuelRelationRejectReason::MissingFuelData:
        return "missingFuelData";
    case FuelRelationRejectReason::NoSeed:
        return "noSeed";
    case FuelRelationRejectReason::FuelNotRunnable:
        return "fuelNotRunnable";
    }
    return "unknown";
}

} // namespace ncfr
