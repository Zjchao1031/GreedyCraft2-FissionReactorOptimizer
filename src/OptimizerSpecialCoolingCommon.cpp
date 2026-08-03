#include "OptimizerSpecialCooling.h"

#include "OptimizerCommon.h"
#include "OptimizerConductorBridge.h"
#include "OptimizerDiagnostics.h"
#include "OptimizerDirectional.h"
#include "FuelSpecialCases.h"
#include "Perf.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
namespace ncfr::optimizer_detail {

#ifndef NDEBUG
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
    , HighHeatPlacementFailureStats* debugStats
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
    const SimulationOptions& simulationOptions
#ifndef NDEBUG
    , HighHeatPlacementFailureStats* debugStats
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
    const SimulationOptions& simulationOptions
#ifndef NDEBUG
    , HighHeatPlacementFailureStats* debugStats
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
    const std::string& detail) {
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
    const std::string& detail) {
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
    const std::string& detail,
    const HighHeatPlacementFailureStats* stats) {
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
    long long initialDeficit, const std::string& detail) {
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

bool hasFunctionalSpecialManaDustCornerSinks(
    const Grid& grid, const FuelSimulation& sim) {
    if (grid.internalA() < 2 || grid.internalB() < 2 ||
        grid.internalC() < 2 ||
        sim.validSinks.size() != static_cast<size_t>(grid.volume()) ||
        sim.heatingClusterBlocks.size() !=
            static_cast<size_t>(grid.volume())) {
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


std::optional<Grid> padMixedFuelGridForSpecialCooling(
    const Grid& grid, const BuildRequest& request,
    const std::vector<FuelLayoutContext>& fuelContexts,
    std::vector<FuelLayoutContext>& paddedContexts
#ifndef NDEBUG
    , const char** debugFailure
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

ManaDustPreparationResult prepareManaDustFallbackGrid(
    const Grid& grid, const BuildRequest& request,
    const std::vector<FuelLayoutContext>& fuelContexts,
    ManaDustCompactionStrategy strategy) {
    std::vector<FuelLayoutContext> paddedContexts;
#ifndef NDEBUG
    const char* paddingFailure = "unknown";
#endif
    std::optional<Grid> padded = padMixedFuelGridForSpecialCooling(
        grid, request, fuelContexts, paddedContexts
#ifndef NDEBUG
        , &paddingFailure
#endif
        );
    if (!padded.has_value()) {
        return {
            std::nullopt,
            ManaDustPreparationFailure::Padding,
#ifndef NDEBUG
            paddingFailure,
#else
            {},
#endif
        };
    }

#ifndef NDEBUG
    std::string preplacementFailure;
#endif
    std::optional<Grid> preplaced =
        tryPreplaceInsetManaDustFallback(
            std::move(*padded)
#ifndef NDEBUG
            , &preplacementFailure
#endif
        );
    if (!preplaced.has_value()) {
        return {
            std::nullopt,
            ManaDustPreparationFailure::Preplacement,
#ifndef NDEBUG
            std::move(preplacementFailure),
#else
            {},
#endif
        };
    }

    std::optional<Grid> compacted;
    if (strategy == ManaDustCompactionStrategy::DirectionalSingleFuel) {
        if (paddedContexts.size() != 1) {
            return {
                std::nullopt,
                ManaDustPreparationFailure::Compaction,
                "singleFuelContextCountInvalid",
            };
        }
        const FuelLayoutContext& context = paddedContexts.front();
        compacted = compactInteriorPlanesPreservingSources(
            *preplaced, request, context.sourceDirections,
            context.fuelLines, 0, true);
    } else {
        compacted = compactInteriorPlanesPreservingSourceTargets(
            *preplaced, 0, true);
    }
    if (!compacted.has_value()) {
        return {
            std::nullopt,
            ManaDustPreparationFailure::Compaction,
            "sourcePreservingCompactionFailed",
        };
    }
    return {
        std::move(compacted),
        ManaDustPreparationFailure::None,
        {},
    };
}


} // namespace ncfr::optimizer_detail
