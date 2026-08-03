#pragma once

#ifndef NDEBUG
#include <chrono>
#endif
#include <cstdint>

namespace ncfr {

struct OptimizationResult;
struct BuildRequest;

namespace perf {

struct Counters {
    long long totalNs = 0;
    long long candidateGenerationNs = 0;
    long long fillSupportNs = 0;
    long long optimisticSinkNs = 0;
    long long improveNs = 0;
    long long targetFuelNs = 0;
    long long scoreNs = 0;
    long long cleanupNs = 0;
    long long simulateFuelNs = 0;
    long long evaluateValidSinksNs = 0;
    long long candidateEvaluationNs = 0;
    long long preparedDedupNs = 0;
    long long compactInteriorPlanesNs = 0;
    long long placeSourcesNs = 0;
    long long allSourcesTargetFuelNs = 0;
    long long hasNoEmptyInteriorPlaneNs = 0;
    long long finalizeCandidateNs = 0;

    long long candidateCount = 0;
    long long candidateEvaluations = 0;
    long long dedupSkippedCandidates = 0;
    long long compactInteriorPlanesCalls = 0;
    long long placeSourcesCalls = 0;
    long long allSourcesTargetFuelCalls = 0;
    long long hasNoEmptyInteriorPlaneCalls = 0;
    long long finalizeCandidateCalls = 0;
    long long bestUpdates = 0;
    long long fillSupportCalls = 0;
    long long optimisticSinkChecks = 0;
    long long optimisticRuleContextBuilds = 0;
    long long improveCalls = 0;
    long long improvePasses = 0;
    long long improveFrontierPositions = 0;
    long long improveTrials = 0;
    long long uncachedTrialSimulations = 0;
    long long improveAcceptedPasses = 0;
    long long targetFuelCalls = 0;
    long long targetFuelChecks = 0;
    long long scoreCalls = 0;
    long long scoreCacheHits = 0;
    long long scoreFullEvaluations = 0;
    long long scoreFuelChecks = 0;
    long long scorePruned = 0;
    long long scoreCacheEvictions = 0;
    long long scoreCacheClears = 0;
    long long cleanupCalls = 0;
    long long cleanupPasses = 0;
    long long cleanupTrials = 0;
    long long cleanupAccepted = 0;
    long long simulateRequests = 0;
    long long simulateCacheHits = 0;
    long long simulateCacheMisses = 0;
    long long simulateCacheEvictions = 0;
    long long simulateCacheClears = 0;
    long long simulateFuelCalls = 0;
    long long simulateFuelIterations = 0;
    long long simulateFuelVolumeTotal = 0;
    long long simulateFuelCellTotal = 0;
    long long traceLineCalls = 0;
    long long evaluateValidSinksCalls = 0;
    long long evaluateValidSinksPasses = 0;
    long long evaluateValidSinksPositions = 0;
    long long sinkValidityChecks = 0;

#ifndef NDEBUG
    long long mergeLayoutCalls = 0;
    long long mergeNoHeatingSinkRejects = 0;
    long long mergeCandidateAttempts = 0;
    long long mergePlanarCandidateAttempts = 0;
    long long mergeAnyAxisCandidateAttempts = 0;
    long long mergeBuildRejectEmpty = 0;
    long long mergeBuildRejectSize = 0;
    long long mergeBuildRejectConflict = 0;
    long long mergeBuildRejectFuelSlot = 0;
    long long mergeBuildRejectFuelDuplicate = 0;
    long long mergeBuildRejectFuelMissing = 0;
    long long mergeBuildRejectSource = 0;
    long long mergeSimulationRejectNotRunnable = 0;
    long long mergeSimulationRejectUnsafeFlux = 0;
    long long mergeSimulationRejectDisconnected = 0;
    long long mergeSimulationRejectCooling = 0;
    long long mergeSimulationRejectClusterCount = 0;
    long long mergeSimulationRejectOther = 0;
    long long mergeAcceptedCandidates = 0;
    long long mergeAcceptedPlanarCandidates = 0;
    long long mergeAcceptedAnyAxisCandidates = 0;
    long long mergeSourcePlacementAttempts = 0;
    long long mergeSourceBoundaryRejects = 0;
    long long mergeSourceLineRejects = 0;
    long long mergeSourceTargetRejects = 0;
    long long mergeSourcePlaced = 0;
#endif
};

#ifndef NDEBUG

using Clock = std::chrono::steady_clock;

Counters& counters();
void resetCounters();
long long elapsedNs(Clock::time_point start);
void logCheckpoint(const char* name, const char* detail);
void appendLatestLog(const BuildRequest& request, const OptimizationResult& result);
void appendFailureLog(const BuildRequest& request, const char* errorType, const char* message);

class ScopeTimer {
public:
    explicit ScopeTimer(long long& targetNs);
    ~ScopeTimer();

    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;

private:
    long long& targetNs_;
    Clock::time_point start_;
};

#endif

} // namespace perf
} // namespace ncfr

#ifndef NDEBUG
#define NCFR_PERF_CONCAT_IMPL(a, b) a##b
#define NCFR_PERF_CONCAT(a, b) NCFR_PERF_CONCAT_IMPL(a, b)
#define NCFR_PERF_SCOPE(field) ::ncfr::perf::ScopeTimer NCFR_PERF_CONCAT(ncfrPerfScope_, __LINE__)(::ncfr::perf::counters().field)
#define NCFR_PERF_COUNT(field) (++::ncfr::perf::counters().field)
#define NCFR_PERF_ADD(field, value) (::ncfr::perf::counters().field += static_cast<long long>(value))
#define NCFR_PERF_CHECKPOINT(name, detail) ::ncfr::perf::logCheckpoint((name), (detail))
#else
#define NCFR_PERF_SCOPE(field) do {} while (false)
#define NCFR_PERF_COUNT(field) do {} while (false)
#define NCFR_PERF_ADD(field, value) do {} while (false)
#define NCFR_PERF_CHECKPOINT(name, detail) do {} while (false)
#endif
