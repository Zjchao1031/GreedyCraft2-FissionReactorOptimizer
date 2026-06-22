#include "Optimizer.h"

#include "OptimizerDetail.h"
#include "Perf.h"

#include <exception>
#include <new>
#include <stdexcept>

namespace ncfr {
namespace {

using namespace optimizer_detail;

class OptimizationStrategy {
public:
    virtual ~OptimizationStrategy() = default;
    virtual OptimizationResult optimize(const BuildRequest& request, const std::atomic_bool* cancelRequested) const = 0;
};

class SingleSelfStartingFuelStrategy final : public OptimizationStrategy {
public:
    OptimizationResult optimize(const BuildRequest& request, const std::atomic_bool* cancelRequested) const override {
        return optimizeSingleFuelDirectionalLayout(request, {{}}, cancelRequested);
    }
};

class SingleNonSelfStartingFuelStrategy final : public OptimizationStrategy {
public:
    OptimizationResult optimize(const BuildRequest& request, const std::atomic_bool* cancelRequested) const override {
        const int sourceCount = requiredSourceCountForFuels(request);
        if (sourceCount > static_cast<int>(kSourceDirections.size())) {
            throw std::runtime_error("单燃料中心策略最多支持 6 个正对燃料的外壁中子源。");
        }

        const std::vector<std::vector<int>> sourceCombos = sourceDirectionCombinations(sourceCount);

        return optimizeSingleFuelDirectionalLayout(request, sourceCombos, cancelRequested);
    }
};

class DualFuelStrategy final : public OptimizationStrategy {
public:
    OptimizationResult optimize(const BuildRequest& request, const std::atomic_bool* cancelRequested) const override {
        return optimizeDualFuelLayout(request, cancelRequested);
    }
};

class QuadFuelStrategy final : public OptimizationStrategy {
public:
    OptimizationResult optimize(const BuildRequest& request, const std::atomic_bool* cancelRequested) const override {
        return optimizeQuadFuelLayout(request, cancelRequested);
    }
};

class SixFuelIrradiatorStrategy final : public OptimizationStrategy {
public:
    OptimizationResult optimize(const BuildRequest& request, const std::atomic_bool* cancelRequested) const override {
        return optimizeSixFuelIrradiatorLayout(request, cancelRequested);
    }
};

class UnimplementedStrategy final : public OptimizationStrategy {
public:
    OptimizationResult optimize(const BuildRequest&, const std::atomic_bool*) const override {
        throw std::runtime_error("该燃料数量/类型的优化策略暂未实现。");
    }
};

const OptimizationStrategy& selectStrategy(const BuildRequest& request) {
    static const SingleSelfStartingFuelStrategy singleSelfStartingFuelStrategy;
    static const SingleNonSelfStartingFuelStrategy singleNonSelfStartingFuelStrategy;
    static const DualFuelStrategy dualFuelStrategy;
    static const QuadFuelStrategy quadFuelStrategy;
    static const SixFuelIrradiatorStrategy sixFuelIrradiatorStrategy;
    static const UnimplementedStrategy unimplementedStrategy;

    if (request.fuelIndices.size() == 1) {
        return fuels().at(static_cast<size_t>(request.fuelIndices.front())).selfPriming
            ? static_cast<const OptimizationStrategy&>(singleSelfStartingFuelStrategy)
            : static_cast<const OptimizationStrategy&>(singleNonSelfStartingFuelStrategy);
    }
    if (request.fuelIndices.size() == 2) {
        return dualFuelStrategy;
    }
    if (request.fuelIndices.size() == 4) {
        return quadFuelStrategy;
    }
    if (request.fuelIndices.size() == 5) {
        return sixFuelIrradiatorStrategy;
    }
    return unimplementedStrategy;
}

} // namespace

int requiredSourceCount(const BuildRequest& request) {
    return requiredSourceCountForFuels(request);
}

OptimizationResult optimizeLayout(const BuildRequest& request, const std::atomic_bool* cancelRequested) {
#ifndef NDEBUG
    perf::resetCounters();
    const auto totalStart = perf::Clock::now();
#endif
    try {
        validateRequest(request);
        throwIfCancelled(cancelRequested);
        OptimizationResult finalResult = selectStrategy(request).optimize(request, cancelRequested);
#ifndef NDEBUG
        perf::counters().totalNs = perf::elapsedNs(totalStart);
        perf::appendLatestLog(request, finalResult);
#endif
        return finalResult;
#ifndef NDEBUG
    } catch (const OptimizationCanceled& ex) {
        perf::counters().totalNs = perf::elapsedNs(totalStart);
        perf::appendFailureLog(request, "canceled", ex.what());
        throw;
    } catch (const std::bad_alloc& ex) {
        perf::counters().totalNs = perf::elapsedNs(totalStart);
        perf::appendFailureLog(request, "std::bad_alloc", ex.what());
        throw;
    } catch (const std::exception& ex) {
        perf::counters().totalNs = perf::elapsedNs(totalStart);
        perf::appendFailureLog(request, "std::exception", ex.what());
        throw;
    } catch (...) {
        perf::counters().totalNs = perf::elapsedNs(totalStart);
        perf::appendFailureLog(request, "unknown", "");
        throw;
    }
#else
    } catch (...) {
        throw;
    }
#endif
}

} // namespace ncfr
