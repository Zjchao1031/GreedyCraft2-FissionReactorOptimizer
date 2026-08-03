#pragma once

#include "OptimizerTypes.h"

namespace ncfr::optimizer_detail {

#ifndef NDEBUG
std::string dimensionLabel(const Dimension& dim);
std::string gridInteriorLabel(const Grid& grid);
const char* directionLabel(int direction);
std::string directionListLabel(const std::vector<int>& directions);
std::string directionalCandidateDetail(const char* reason, const Dimension& dim,
                                       const std::vector<int>& sourceDirections,
                                       const std::vector<int>& reflectorDirections);
std::string directionalGridDetail(const char* reason, const Grid& grid,
                                  const FuelSimulation* sim,
                                  const BuildRequest& request,
                                  const std::vector<int>& sourceDirections,
                                  const std::vector<int>& reflectorDirections);
std::string fuelRelationDetail(const char* reason,
                               const FuelRelationPrefilterResult& result,
                               const BuildRequest& request);
void logFinalizeCheckpoint(const char* checkpointName, const std::string& detail,
                           int paddingPlanes,
                           const ImproveOptions& improveOptions);
void logCoolingExpansionCheckpoint(const char* reason, const Grid& grid,
                                   const FuelSimulation& sim, int pass = -1,
                                   const Pos* pos = nullptr,
                                   const SinkType* sink = nullptr,
                                   long long oldMargin = 0,
                                   const Pos* bridgePos = nullptr,
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

void recordCoolingExpansionRejection(CoolingExpansionPassStats& stats,
                                     const char* reason,
                                     const FuelSimulation* sim = nullptr);
void logCoolingExpansionStats(const char* reason, const Grid& grid,
                              const FuelSimulation& sim, int pass,
                              const CoolingExpansionPassStats& stats);
#endif

} // namespace ncfr::optimizer_detail
