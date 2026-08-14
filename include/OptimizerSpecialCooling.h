#pragma once

#include "OptimizerTypes.h"

#include <sstream>

namespace ncfr::optimizer_detail {

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

std::string posLabel(const Pos& pos);
const char* blockKindLabel(BlockKind kind);
void appendHighHeatFailureStats(std::ostringstream& os, const char* prefix,
                                const HighHeatPlacementFailureStats& stats);
void logHighHeatCoolingCheckpoint(const char* reason, const Grid& grid,
                                  const FuelSimulation& sim,
                                  size_t endStoneCandidates,
                                  size_t endStonePlaced,
                                  size_t endStoneOccupied,
                                  size_t endStoneFailed,
                                  size_t carobbiiteCandidates,
                                  size_t carobbiitePlaced,
                                  size_t carobbiiteFailed,
                                  size_t manaDustSinks);
void logHighHeatPlacementFailures(const char* sinkName, const Grid& grid,
                                  const HighHeatPlacementFailureStats& stats,
                                  const std::string& detail = {});
void logHighHeatFinalReview(const char* reason, const Grid& grid,
                            const FuelSimulation& sim, bool accepted,
                            bool endStoneFunctional,
                            bool carobbiiteFunctional,
                            size_t endStoneFaces,
                            size_t carobbiiteFaces);
void logDualFuelCoolingCheckpoint(const char* reason, const Grid& grid,
                                  const FuelSimulation& sim,
                                  long long initialDeficit,
                                  bool allowCarobbiite, bool allowManaDust,
                                  const std::string& detail = {});
void logDualFuelSinkCheckpoint(
    const char* stage, int requestSlot, const Pos& pos, int faceDirection,
    const char* outcome, const Grid& grid, const std::string& detail = {},
    const HighHeatPlacementFailureStats* stats = nullptr);
void logDualFuelFallbackCheckpoint(const char* reason, const Grid& grid,
                                   const FuelSimulation& sim,
                                   long long initialDeficit,
                                   const std::string& detail = {});
#endif

enum class ManaDustCompactionStrategy {
    DirectionalSingleFuel,
    PreserveMixedFuelSources,
};

enum class ManaDustPreparationFailure {
    None,
    Padding,
    Preplacement,
    Compaction,
};

struct ManaDustPreparationResult {
    std::optional<Grid> grid;
    ManaDustPreparationFailure failure = ManaDustPreparationFailure::None;
    std::string detail;
};

bool isSpecialManaDustRequest(const BuildRequest& request);
bool hasSpecialManaDustCoolingDeficit(const FuelSimulation& sim);
bool isHighHeatSingleFuelFallbackEligible(const BuildRequest& request,
                                          const FuelSimulation& sim);
int endStoneSinkType();
int carobbiiteSinkType();
bool isEndStoneSink(const Block& block);
bool isCarobbiiteSink(const Block& block);
void markDirectionalLayoutProtected(
    StateVector& protectedPositions, const Grid& grid, const Pos& fuelPos,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines);
void markOccupiedInteriorProtected(StateVector& protectedPositions,
                                   const Grid& grid);
std::optional<std::vector<EndStoneReflectorCandidate>>
endStoneReflectorSinkCandidates(const Grid& grid, const Pos& fuelPos,
                                const std::vector<FuelLineSpec>& fuelLines);
std::vector<CarobbiiteReflectorCandidate> carobbiiteReflectorSinkCandidates(
    const Grid& grid, const std::vector<FuelLineSpec>& fuelLines,
    const std::vector<EndStoneReflectorCandidate>& endStoneCandidates);
bool tryPlaceCarobbiiteSink(
    Grid& grid, FuelSimulation& currentSim, StateVector& protectedPositions,
    const CarobbiiteReflectorCandidate& candidate
#ifndef NDEBUG
    , HighHeatPlacementFailureStats* debugStats = nullptr
#endif
);
void removeCarobbiiteSinksForEndStone(
    Grid& grid, StateVector& protectedPositions,
    std::vector<int>& placedCarobbiiteFaces,
    const std::vector<CarobbiiteReflectorCandidate>& carobbiiteCandidates,
    const Pos& endStonePos);
bool tryConnectSpecialSinkToHeatingCluster(
    Grid& grid, FuelSimulation& currentSim, const Pos& sinkPos,
    StateVector& protectedPositions, const std::atomic_bool* cancelRequested,
    const SimulationOptions& simulationOptions = {}
#ifndef NDEBUG
    , HighHeatPlacementFailureStats* debugStats = nullptr
#endif
);
bool connectSpecialSinksToHeatingCluster(
    Grid& grid, FuelSimulation& currentSim,
    const std::vector<Pos>& sinkPositions,
    StateVector& protectedPositions, const std::atomic_bool* cancelRequested,
    const SimulationOptions& simulationOptions = {}
#ifndef NDEBUG
    , HighHeatPlacementFailureStats* debugStats = nullptr
#endif
);
std::vector<Pos> placedSpecialCoolingSinkPositions(const Grid& grid);
bool hasFunctionalEndStoneSinks(
    const Grid& grid, const FuelSimulation& sim,
    const std::vector<FuelLineSpec>& fuelLines,
    const std::vector<int>& placedFaceDirections);
bool hasFunctionalSpecialCarobbiiteSinks(
    const Grid& grid, const FuelSimulation& sim,
    const std::vector<FuelLineSpec>& fuelLines,
    const std::vector<int>& placedFaceDirections);
bool hasFunctionalSpecialManaDustCornerSinks(
    const Grid& grid, const FuelSimulation& sim);
ManaDustPreparationResult prepareManaDustFallbackGrid(
    const Grid& grid, const BuildRequest& request,
    const std::vector<FuelLayoutContext>& fuelContexts,
    ManaDustCompactionStrategy strategy);
std::optional<FinalizeResult> trySpecialManaDustFinalization(
    const Grid& grid, const FuelSimulation& sim, const BuildRequest& request,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines,
    const StateVector* protectedPositions,
    const std::atomic_bool* cancelRequested);
std::optional<FinalizeResult> tryHighHeatSingleFuelFinalization(
    const Grid& grid, const FuelSimulation& sim, const BuildRequest& request,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines,
    const std::atomic_bool* cancelRequested);
std::optional<Grid> tryMixedFuelSpecialCoolingFallback(
    Grid grid, const BuildRequest& request,
    const std::vector<FuelLayoutContext>& fuelContexts,
    const std::atomic_bool* cancelRequested,
    bool allowDisconnectedFunctionalBlocks = false,
    CoolingValidationPolicy coolingPolicy =
        CoolingValidationPolicy::PerCluster,
    std::optional<long long> manaDustCoolingHandoffThreshold =
        std::nullopt);

} // namespace ncfr::optimizer_detail
