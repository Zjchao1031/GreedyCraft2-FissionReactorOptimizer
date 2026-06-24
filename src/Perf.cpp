#include "Perf.h"

#ifndef NDEBUG

#include "Optimizer.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <deque>
#include <sstream>
#include <string>

namespace ncfr::perf {
namespace {

Counters gCounters;
std::deque<std::string> gCheckpoints;
long long gDroppedCheckpoints = 0;

constexpr size_t kMaxCheckpoints = 200;

std::filesystem::path findProjectDirectoryFrom(std::filesystem::path path) {
    if (path.empty()) {
        return {};
    }
    std::error_code ec;
    if (std::filesystem::is_regular_file(path, ec)) {
        path = path.parent_path();
    }
    path = std::filesystem::absolute(path, ec);
    while (!path.empty()) {
        if (std::filesystem::exists(path / "CMakeLists.txt", ec)) {
            return path;
        }
        const std::filesystem::path parent = path.parent_path();
        if (parent == path) {
            break;
        }
        path = parent;
    }
    return {};
}

std::filesystem::path projectDirectory() {
    if (const std::filesystem::path fromCurrent = findProjectDirectoryFrom(std::filesystem::current_path());
        !fromCurrent.empty()) {
        return fromCurrent;
    }
    return {};
}

double ms(long long ns) {
    return static_cast<double>(ns) / 1000000.0;
}

std::string timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#ifdef _WIN32
    localtime_s(&localTime, &time);
#else
    localtime_r(&time, &localTime);
#endif
    std::ostringstream os;
    os << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S");
    return os.str();
}

void writeMetric(std::ostream& os, const char* name, long long value) {
    os << name << ": " << value << '\n';
}

void writeTimeMetric(std::ostream& os, const char* name, long long ns) {
    os << name << ": " << std::fixed << std::setprecision(3) << ms(ns) << " ms\n";
}

std::filesystem::path latestLogPath() {
    const std::filesystem::path dir = projectDirectory();
    return (dir.empty() ? std::filesystem::current_path() : dir) / "latest.log";
}

void writeCounters(std::ostream& os) {
    const Counters& metrics = counters();
    writeTimeMetric(os, "time.total", metrics.totalNs);
    writeTimeMetric(os, "time.candidateGeneration", metrics.candidateGenerationNs);
    writeTimeMetric(os, "time.fillSupport", metrics.fillSupportNs);
    writeTimeMetric(os, "time.optimisticSink", metrics.optimisticSinkNs);
    writeTimeMetric(os, "time.improve", metrics.improveNs);
    writeTimeMetric(os, "time.targetFuel", metrics.targetFuelNs);
    writeTimeMetric(os, "time.score", metrics.scoreNs);
    writeTimeMetric(os, "time.cleanup", metrics.cleanupNs);
    writeTimeMetric(os, "time.simulateFuel", metrics.simulateFuelNs);
    writeTimeMetric(os, "time.evaluateValidSinks", metrics.evaluateValidSinksNs);
    writeTimeMetric(os, "time.candidateEvaluation", metrics.candidateEvaluationNs);
    writeTimeMetric(os, "time.preparedDedup", metrics.preparedDedupNs);
    writeTimeMetric(os, "time.compactInteriorPlanes", metrics.compactInteriorPlanesNs);
    writeTimeMetric(os, "time.placeSources", metrics.placeSourcesNs);
    writeTimeMetric(os, "time.allSourcesTargetFuel", metrics.allSourcesTargetFuelNs);
    writeTimeMetric(os, "time.hasNoEmptyInteriorPlane", metrics.hasNoEmptyInteriorPlaneNs);
    writeTimeMetric(os, "time.finalizeCandidate", metrics.finalizeCandidateNs);
    writeMetric(os, "count.candidates", metrics.candidateCount);
    writeMetric(os, "count.candidateEvaluations", metrics.candidateEvaluations);
    writeMetric(os, "count.dedupSkippedCandidates", metrics.dedupSkippedCandidates);
    writeMetric(os, "count.compactInteriorPlanesCalls", metrics.compactInteriorPlanesCalls);
    writeMetric(os, "count.placeSourcesCalls", metrics.placeSourcesCalls);
    writeMetric(os, "count.allSourcesTargetFuelCalls", metrics.allSourcesTargetFuelCalls);
    writeMetric(os, "count.hasNoEmptyInteriorPlaneCalls", metrics.hasNoEmptyInteriorPlaneCalls);
    writeMetric(os, "count.finalizeCandidateCalls", metrics.finalizeCandidateCalls);
    writeMetric(os, "count.bestUpdates", metrics.bestUpdates);
    writeMetric(os, "count.fillSupportCalls", metrics.fillSupportCalls);
    writeMetric(os, "count.optimisticSinkChecks", metrics.optimisticSinkChecks);
    writeMetric(os, "count.optimisticRuleContextBuilds", metrics.optimisticRuleContextBuilds);
    writeMetric(os, "count.improveCalls", metrics.improveCalls);
    writeMetric(os, "count.improvePasses", metrics.improvePasses);
    writeMetric(os, "count.improveFrontierPositions", metrics.improveFrontierPositions);
    writeMetric(os, "count.improveTrials", metrics.improveTrials);
    writeMetric(os, "count.uncachedTrialSimulations", metrics.uncachedTrialSimulations);
    writeMetric(os, "count.improveAcceptedPasses", metrics.improveAcceptedPasses);
    writeMetric(os, "count.targetFuelCalls", metrics.targetFuelCalls);
    writeMetric(os, "count.targetFuelChecks", metrics.targetFuelChecks);
    writeMetric(os, "count.scoreCalls", metrics.scoreCalls);
    writeMetric(os, "count.scoreCacheHits", metrics.scoreCacheHits);
    writeMetric(os, "count.scoreFullEvaluations", metrics.scoreFullEvaluations);
    writeMetric(os, "count.scoreFuelChecks", metrics.scoreFuelChecks);
    writeMetric(os, "count.scorePruned", metrics.scorePruned);
    writeMetric(os, "count.scoreCacheEvictions", metrics.scoreCacheEvictions);
    writeMetric(os, "count.scoreCacheClears", metrics.scoreCacheClears);
    writeMetric(os, "count.cleanupCalls", metrics.cleanupCalls);
    writeMetric(os, "count.cleanupPasses", metrics.cleanupPasses);
    writeMetric(os, "count.cleanupTrials", metrics.cleanupTrials);
    writeMetric(os, "count.cleanupAccepted", metrics.cleanupAccepted);
    writeMetric(os, "count.simulateRequests", metrics.simulateRequests);
    writeMetric(os, "count.simulateCacheHits", metrics.simulateCacheHits);
    writeMetric(os, "count.simulateCacheMisses", metrics.simulateCacheMisses);
    writeMetric(os, "count.simulateCacheEvictions", metrics.simulateCacheEvictions);
    writeMetric(os, "count.simulateCacheClears", metrics.simulateCacheClears);
    writeMetric(os, "count.simulateFuelCalls", metrics.simulateFuelCalls);
    writeMetric(os, "count.simulateFuelIterations", metrics.simulateFuelIterations);
    writeMetric(os, "count.simulateFuelVolumeTotal", metrics.simulateFuelVolumeTotal);
    writeMetric(os, "count.simulateFuelCellTotal", metrics.simulateFuelCellTotal);
    writeMetric(os, "count.traceLineCalls", metrics.traceLineCalls);
    writeMetric(os, "count.evaluateValidSinksCalls", metrics.evaluateValidSinksCalls);
    writeMetric(os, "count.evaluateValidSinksPasses", metrics.evaluateValidSinksPasses);
    writeMetric(os, "count.evaluateValidSinksPositions", metrics.evaluateValidSinksPositions);
    writeMetric(os, "count.sinkValidityChecks", metrics.sinkValidityChecks);
    writeMetric(os, "count.mergeLayoutCalls", metrics.mergeLayoutCalls);
    writeMetric(os, "count.mergeNoHeatingSinkRejects", metrics.mergeNoHeatingSinkRejects);
    writeMetric(os, "count.mergeCandidateAttempts", metrics.mergeCandidateAttempts);
    writeMetric(os, "count.mergePlanarCandidateAttempts", metrics.mergePlanarCandidateAttempts);
    writeMetric(os, "count.mergeAnyAxisCandidateAttempts", metrics.mergeAnyAxisCandidateAttempts);
    writeMetric(os, "count.mergeBuildRejectEmpty", metrics.mergeBuildRejectEmpty);
    writeMetric(os, "count.mergeBuildRejectSize", metrics.mergeBuildRejectSize);
    writeMetric(os, "count.mergeBuildRejectConflict", metrics.mergeBuildRejectConflict);
    writeMetric(os, "count.mergeBuildRejectFuelSlot", metrics.mergeBuildRejectFuelSlot);
    writeMetric(os, "count.mergeBuildRejectFuelDuplicate", metrics.mergeBuildRejectFuelDuplicate);
    writeMetric(os, "count.mergeBuildRejectFuelMissing", metrics.mergeBuildRejectFuelMissing);
    writeMetric(os, "count.mergeBuildRejectSource", metrics.mergeBuildRejectSource);
    writeMetric(os, "count.mergeSimulationRejectNotRunnable", metrics.mergeSimulationRejectNotRunnable);
    writeMetric(os, "count.mergeSimulationRejectUnsafeFlux", metrics.mergeSimulationRejectUnsafeFlux);
    writeMetric(os, "count.mergeSimulationRejectDisconnected", metrics.mergeSimulationRejectDisconnected);
    writeMetric(os, "count.mergeSimulationRejectCooling", metrics.mergeSimulationRejectCooling);
    writeMetric(os, "count.mergeSimulationRejectClusterCount", metrics.mergeSimulationRejectClusterCount);
    writeMetric(os, "count.mergeSimulationRejectOther", metrics.mergeSimulationRejectOther);
    writeMetric(os, "count.mergeAcceptedCandidates", metrics.mergeAcceptedCandidates);
    writeMetric(os, "count.mergeAcceptedPlanarCandidates", metrics.mergeAcceptedPlanarCandidates);
    writeMetric(os, "count.mergeAcceptedAnyAxisCandidates", metrics.mergeAcceptedAnyAxisCandidates);
    writeMetric(os, "count.mergeSourcePlacementAttempts", metrics.mergeSourcePlacementAttempts);
    writeMetric(os, "count.mergeSourceBoundaryRejects", metrics.mergeSourceBoundaryRejects);
    writeMetric(os, "count.mergeSourceLineRejects", metrics.mergeSourceLineRejects);
    writeMetric(os, "count.mergeSourceTargetRejects", metrics.mergeSourceTargetRejects);
    writeMetric(os, "count.mergeSourcePlaced", metrics.mergeSourcePlaced);
}

void writeCheckpoints(std::ostream& os) {
    writeMetric(os, "checkpoint.count", static_cast<long long>(gCheckpoints.size()));
    writeMetric(os, "checkpoint.dropped", gDroppedCheckpoints);
    for (const std::string& checkpoint : gCheckpoints) {
        os << "checkpoint: " << checkpoint << '\n';
    }
}

} // namespace

Counters& counters() {
    return gCounters;
}

void resetCounters() {
    gCounters = {};
    gCheckpoints.clear();
    gDroppedCheckpoints = 0;
}

long long elapsedNs(Clock::time_point start) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count();
}

void logCheckpoint(const char* name, const char* detail) {
    std::ostringstream os;
    os << (name == nullptr ? "unknown" : name);
    if (detail != nullptr && detail[0] != '\0') {
        os << " " << detail;
    }
    if (gCheckpoints.size() >= kMaxCheckpoints) {
        gCheckpoints.pop_front();
        ++gDroppedCheckpoints;
    }
    gCheckpoints.push_back(os.str());
}

ScopeTimer::ScopeTimer(long long& targetNs) : targetNs_(targetNs), start_(Clock::now()) {}

ScopeTimer::~ScopeTimer() {
    targetNs_ += elapsedNs(start_);
}

void appendLatestLog(const BuildRequest& request, const OptimizationResult& result) {
    std::ofstream os(latestLogPath(), std::ios::app);
    if (!os) {
        return;
    }

    os << "==== FissionReactor Debug Performance " << timestamp() << " ====\n";
    os << "status: success\n";
    os << "requestFuelCells: " << request.fuelIndices.size() << '\n';
    os << "requestSources: " << requiredSourceCount(request) << '\n';
    os << "resultSize: " << result.grid.internalA() << "x" << result.grid.internalB() << "x"
       << result.grid.internalC() << '\n';
    os << "resultMinCoolingMargin: " << result.minCoolingMargin << '\n';
    os << "resultDisconnectedFunctionalBlocks: " << result.disconnectedFunctionalBlocks << '\n';
    writeCounters(os);
    writeCheckpoints(os);
    os << '\n';
}

void appendFailureLog(const BuildRequest& request, const char* errorType, const char* message) {
    std::ofstream os(latestLogPath(), std::ios::app);
    if (!os) {
        return;
    }

    os << "==== FissionReactor Debug Performance " << timestamp() << " ====\n";
    os << "status: failed\n";
    os << "requestFuelCells: " << request.fuelIndices.size() << '\n';
    os << "requestSources: " << requiredSourceCount(request) << '\n';
    os << "errorType: " << (errorType == nullptr ? "unknown" : errorType) << '\n';
    os << "errorMessage: " << (message == nullptr ? "" : message) << '\n';
    writeCounters(os);
    writeCheckpoints(os);
    os << '\n';
}

} // namespace ncfr::perf

#endif
