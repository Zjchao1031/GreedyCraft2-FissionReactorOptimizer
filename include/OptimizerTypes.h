#pragma once

#include "FuelPlacementPrefilter.h"
#include "Optimizer.h"
#include "ReactorMetrics.h"
#include "Rule.h"
#include "Simulator.h"
#include "StateVector.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace ncfr::optimizer_detail {

inline constexpr int kMaxSize = 24;

struct Dimension { int a = 1; int b = 1; int c = 1; };
struct CandidateScore {
    bool compatible = false;
    bool safeFlux = false;
    long long minCoolingMargin = 0;
    int disconnectedFunctionalBlocks = 0;
    int functionalIrradiators = 0;
    int usefulBlocks = 0;
    long long cooling = 0;
};
struct ImproveOptions { int maxPasses = 18; int frontierRadius = 1; size_t frontierLimit = 360; };
struct CoolingExpansionOptions {
    int maxPasses = 24;
    int radius = 3;
    size_t positionLimit = 512;
    size_t sinkTypeLimit = 4;
    size_t bridgeTargetLimit = 512;
    size_t bridgeTargetCandidateLimit = 128;
    size_t bridgeCandidateLimit = 128;
    size_t bridgeSinkTypeLimit = 4;
    long long handoffCoolingDeficit = -1;
};
struct SupportBlockOptions {
    std::vector<int> moderatorTypeIndices;
    std::vector<int> reflectorTypeIndices;
};
struct SourcePrimingTarget { Pos source; int targetIndex = -1; };
struct FuelLineSpec {
    int direction = 0;
    int moderatorCount = 1;
    int moderatorType = 0;
    int reflectorType = 0;
    double estimatedFlux = 0.0;
};
struct MergeableSingleFuelLayout {
    Grid grid;
    FuelSimulation sim;
    std::vector<int> sourceDirections;
    std::vector<FuelLineSpec> fuelLines;
};
struct MergeableSingleFuelSearchGoal {
    long long minimumCooling = 0;
    long long pairedRawHeating = 0;
    long long pairedCooling = 0;
    bool allowCombinedBalance = false;
};
struct FuelLayoutContext {
    int requestSlot = -1;
    Pos fuelPos;
    std::vector<int> sourceDirections;
    std::vector<FuelLineSpec> fuelLines;
};
enum class FinalizeFailureKind {
    None, NotRunnable, UnsafeFlux, CoolingDeficit, Disconnected,
    WallDisconnected, Structural
};
struct FinalizeResult {
    std::optional<OptimizationResult> result;
    FinalizeFailureKind failure = FinalizeFailureKind::None;
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
struct Direction { int dx = 0; int dy = 0; int dz = 0; };

inline constexpr ImproveOptions kDefaultImproveOptions{};
inline constexpr CoolingExpansionOptions kCoolingExpansionOptions{};
inline constexpr std::array<Direction, 6> kSourceDirections = {{
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
    {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
}};

} // namespace ncfr::optimizer_detail
