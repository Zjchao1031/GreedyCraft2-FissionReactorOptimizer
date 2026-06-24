#pragma once

#include "FuelPlacementPrefilter.h"
#include "Optimizer.h"
#include "Rule.h"
#include "Simulator.h"
#include "StateVector.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace ncfr::optimizer_detail {

inline constexpr int kMaxSize = 24;

struct Dimension {
    int a = 1;
    int b = 1;
    int c = 1;
};

struct CandidateScore {
    bool compatible = false;
    bool safeFlux = false;
    long long minCoolingMargin = 0;
    int disconnectedFunctionalBlocks = 0;
    int functionalIrradiators = 0;
    int usefulBlocks = 0;
    long long cooling = 0;
};

struct ImproveOptions {
    int maxPasses = 18;
    int frontierRadius = 1;
    size_t frontierLimit = 360;
};

struct CoolingExpansionOptions {
    int maxPasses = 24;
    int radius = 3;
    size_t positionLimit = 512;
    size_t sinkTypeLimit = 4;
    size_t bridgeTargetLimit = 512;
    size_t bridgeTargetCandidateLimit = 128;
    size_t bridgeCandidateLimit = 128;
    size_t bridgeSinkTypeLimit = 4;
};

struct SupportBlockOptions {
    std::vector<int> moderatorTypeIndices;
    std::vector<int> reflectorTypeIndices;
};

struct FuelLineSpec {
    int direction = 0;
    int moderatorCount = 1;
    int moderatorType = 0;
    int reflectorType = 0;
    double estimatedFlux = 0.0;
};

enum class FinalizeFailureKind {
    None,
    NotRunnable,
    UnsafeFlux,
    CoolingDeficit,
    Disconnected,
    Structural
};

struct FinalizeResult {
    std::optional<OptimizationResult> result;
    FinalizeFailureKind failure = FinalizeFailureKind::None;
};

struct Direction {
    int dx = 0;
    int dy = 0;
    int dz = 0;
};

inline constexpr ImproveOptions kDefaultImproveOptions{};
inline constexpr CoolingExpansionOptions kCoolingExpansionOptions{};
inline constexpr std::array<Direction, 6> kSourceDirections = {{
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1},
}};

void throwIfCancelled(const std::atomic_bool* cancelRequested);

int dimensionVolume(const Dimension& dim);
int dimensionSpread(const Dimension& dim);
int dimensionSurface(const Dimension& dim);

void validateRequest(const BuildRequest& request);
int requiredSourceCountForFuels(const BuildRequest& request);

std::vector<Dimension> sortedDimensions();
int adjacentCells(const Grid& grid, const Pos& pos);
bool isBetweenFuelCells(const Grid& grid, const Pos& pos, int axis);
RuleContext optimisticRuleContext(const Grid& grid, StateVector& validSinks, StateVector& functionalCells,
                                  StateVector& activeModerators, StateVector& activeReflectors,
                                  StateVector& functionalShields, StateVector& functionalIrradiators);
bool optimisticSinkValidAt(Grid& grid, const Pos& pos, const RuleContext& context);
int manaDustSinkType();
bool isManaDustSink(const Block& block);
bool isInteriorCorner(const Grid& grid, const Pos& pos);
bool cornerSinkConnectsToInteriorCluster(const Grid& grid, const Pos& corner);
void removeUnclusteredCornerManaDustSinks(Grid& grid);
void fillSupportBlocks(Grid& grid, const SupportBlockOptions* supportOptions = nullptr,
                       const StateVector* protectedPositions = nullptr);
std::vector<Pos> fuelPositionsInGrid(const Grid& grid);
bool allSourcesTargetFuel(const Grid& grid);
bool hasNoEmptyInteriorPlane(const Grid& grid);
std::vector<Block> replacementBlocks(const SupportBlockOptions* supportOptions = nullptr);
bool isSupportMutable(const Block& block);
bool isRequiredSupportBlock(const Grid& grid, const FuelSimulation& sim, int idx);
int countFunctionalIrradiators(const FuelSimulation& sim);
int countUsefulBlocks(const Grid& grid);
std::vector<int> uniqueFuelIndicesInRequest(const BuildRequest& request);
std::vector<Pos> fuelCellPortPositions(const Grid& grid);
void addFuelCellPorts(Grid& grid, const BuildRequest& request);
CandidateScore scoreSimulation(const Grid& grid, const FuelSimulation& sim);
bool betterScore(const CandidateScore& lhs, const CandidateScore& rhs);
std::vector<Pos> improvementPositions(const Grid& grid, const FuelSimulation& sim, const ImproveOptions& options,
                                      const StateVector* protectedPositions = nullptr, bool emptyOnly = false);
Grid improveSupportBlocks(Grid grid, const std::atomic_bool* cancelRequested,
                          const ImproveOptions& options = kDefaultImproveOptions,
                          const SupportBlockOptions* supportOptions = nullptr,
                          const StateVector* protectedPositions = nullptr,
                          bool emptyOnly = false);

Grid expandCooling(Grid grid, const BuildRequest& request, const std::vector<int>& sourceDirections,
                   const std::vector<FuelLineSpec>& fuelLines,
                   const std::atomic_bool* cancelRequested, const CoolingExpansionOptions& options);
Grid expandCoolingWithPreserver(Grid grid, const std::function<bool(Grid&)>& preserveGrid,
                                const std::atomic_bool* cancelRequested,
                                const CoolingExpansionOptions& options,
                                bool allowDisconnectedFunctionalBlocks = false);

OptimizationResult resultFromSimulation(Grid grid, const BuildRequest& request, const FuelSimulation& sim);
bool isAccepted(const Grid& grid, const FuelSimulation& sim);
bool isPreCompactRunnable(const FuelSimulation& sim);
FinalizeFailureKind classifyFinalizationFailure(const Grid& grid, const FuelSimulation& sim,
                                                const BuildRequest& request);
FinalizeFailureKind finalizeFailureFromFuelRelation(const FuelRelationPrefilterResult& result);

bool samePos(const Pos& lhs, const Pos& rhs);
bool containsDirectionIndex(const std::vector<int>& indices, int index);
Pos offset(const Pos& pos, const Direction& dir, int distance);
Pos sourcePositionForDirection(const Grid& grid, const Pos& fuelPos, const Direction& dir);
std::vector<std::vector<int>> sourceDirectionCombinations(int sourceCount);
std::vector<Dimension> singleFuelSearchDimensions(const FuelActivationProfile& profile);
bool placeDirectionalSources(Grid& grid, const BuildRequest& request, const Pos& fuelPos,
                             const std::vector<int>& sourceDirections);
bool isFullyReflectiveReflector(const Block& block);
Block sourceLineReplacementBlock(const BuildRequest& request);
void keepSourceLinesOpen(Grid& grid, const BuildRequest& request, const std::vector<int>& sourceDirections);
bool restoreDirectionalFuelLines(Grid& grid, const BuildRequest& request, const std::vector<int>& sourceDirections,
                                 const std::vector<FuelLineSpec>& fuelLines);
void pruneInactiveSupport(Grid& grid, const StateVector* protectedPositions = nullptr);
FinalizeResult tryFinalizeDirectionalCandidate(Grid grid, const BuildRequest& request,
                                               const std::vector<int>& sourceDirections,
                                               const std::vector<FuelLineSpec>& fuelLines,
                                               const StateVector* protectedPositions,
                                               const std::atomic_bool* cancelRequested);
OptimizationResult optimizeSingleFuelDirectionalLayout(const BuildRequest& request,
                                                       const std::vector<std::vector<int>>& sourceCombos,
                                                       const std::atomic_bool* cancelRequested);
BuildRequest singleFuelRequestForSlot(const BuildRequest& request, int slot);
OptimizationResult optimizeSingleFuelForSlot(const BuildRequest& request, int slot,
                                             const std::atomic_bool* cancelRequested);
bool heatPriorityLess(int lhsSlot, int rhsSlot, const BuildRequest& request);

OptimizationResult optimizeDualFuelLayout(const BuildRequest& request, const std::atomic_bool* cancelRequested);
OptimizationResult optimizeQuadFuelLayout(const BuildRequest& request, const std::atomic_bool* cancelRequested);
OptimizationResult optimizeSixFuelIrradiatorLayout(const BuildRequest& request,
                                                   const std::atomic_bool* cancelRequested);

#ifndef NDEBUG
std::string dimensionLabel(const Dimension& dim);
std::string gridInteriorLabel(const Grid& grid);
const char* directionLabel(int direction);
std::string directionListLabel(const std::vector<int>& directions);
std::string directionalCandidateDetail(const char* reason, const Dimension& dim,
                                       const std::vector<int>& sourceDirections,
                                       const std::vector<int>& reflectorDirections);
std::string directionalGridDetail(const char* reason, const Grid& grid, const FuelSimulation* sim,
                                  const BuildRequest& request, const std::vector<int>& sourceDirections,
                                  const std::vector<int>& reflectorDirections);
std::string fuelRelationDetail(const char* reason, const FuelRelationPrefilterResult& result,
                               const BuildRequest& request);
void logFinalizeCheckpoint(const char* checkpointName, const std::string& detail, int paddingPlanes,
                           const ImproveOptions& improveOptions);
void logCoolingExpansionCheckpoint(const char* reason, const Grid& grid, const FuelSimulation& sim, int pass = -1,
                                   const Pos* pos = nullptr, const SinkType* sink = nullptr,
                                   long long oldMargin = 0, const Pos* bridgePos = nullptr,
                                   const SinkType* bridgeSink = nullptr);

struct CoolingExpansionPassStats {
    size_t positions = 0;
    size_t directPositions = 0;
    size_t bridgeTargetPositions = 0;
    size_t clusterConnectedPositions = 0;
    size_t sinkTypes = 0;
    long long ruleChecks = 0;
    long long ruleValidSinks = 0;
    long long bridgeRuleChecks = 0;
    long long bridgeRuleValidSinks = 0;
    long long singleCandidates = 0;
    long long bridgeTargetCandidates = 0;
    long long bridgeCandidates = 0;
    long long selectedCandidates = 0;
    long long trials = 0;
    long long bridgeTrials = 0;
    long long restoreLineFailed = 0;
    long long invalidNewSink = 0;
    long long invalidBridgeTarget = 0;
    long long invalidBridgeSink = 0;
    long long notRunnable = 0;
    long long disconnected = 0;
    long long noMarginGain = 0;
    long long notBest = 0;
    long long newBest = 0;
    long long bridgeNewBest = 0;
    long long bestRejectedMargin = std::numeric_limits<long long>::min();
    long long bestRejectedCooling = 0;
    const char* bestRejectedReason = "none";
};

void recordCoolingExpansionRejection(CoolingExpansionPassStats& stats, const char* reason,
                                     const FuelSimulation* sim = nullptr);
void logCoolingExpansionStats(const char* reason, const Grid& grid, const FuelSimulation& sim, int pass,
                              const CoolingExpansionPassStats& stats);
#endif

} // namespace ncfr::optimizer_detail
