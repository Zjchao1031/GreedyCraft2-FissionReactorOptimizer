#include "FuelPlacementPrefilter.h"

#include "Data.h"
#include "NeutronRules.h"
#include "NeutronLineTraversal.h"
#include "Optimizer.h"
#include "Simulator.h"
#include "StateVector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ncfr {
namespace {

constexpr double kFluxEpsilon = 1e-9;

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
        for (const Pos& direction : kNeutronLineDirections) {
            const NeutronLineResult result =
                traceNeutronLine(grid, fuel, from, direction);
            switch (result.endpoint) {
            case NeutronLineEndpoint::FuelCell:
                fluxByIndex.at(static_cast<size_t>(result.targetIndex)) +=
                    result.flux;
                break;
            case NeutronLineEndpoint::Irradiator:
                irradiatorFluxByIndex.at(
                    static_cast<size_t>(result.targetIndex)) += result.flux;
                break;
            case NeutronLineEndpoint::Reflector:
                fluxByIndex.at(static_cast<size_t>(idx)) += result.flux;
                break;
            case NeutronLineEndpoint::None:
                break;
            }
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
        const double fullReflectorFlux = std::floor(
            2.0 * kMaxReflectorLineModerators * heavyWaterFlux * reflectorTypes().at(0).reflectivity);
        const double halfReflectorFlux = std::floor(
            2.0 * kMaxReflectorLineModerators * heavyWaterFlux * reflectorTypes().at(1).reflectivity);
        for (int index = 0; index < static_cast<int>(fuels().size()); ++index) {
            const Fuel& fuel = fuels().at(static_cast<size_t>(index));
            result.push_back({index,
                              fuel.criticality,
                              fuel.intrinsicFlux,
                              fuel.heat,
                              fuel.selfPriming,
                              minLineCount(fuel.criticality, fullReflectorFlux),
                              minLineCount(fuel.criticality, halfReflectorFlux),
                              minLineCount(fuel.criticality, heavyWaterFlux)});
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
