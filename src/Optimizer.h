#pragma once

#include "Data.h"
#include "Grid.h"

#include <atomic>
#include <exception>
#include <utility>
#include <vector>

namespace ncfr {

struct BuildRequest {
    std::vector<int> fuelIndices;
};

class OptimizationCanceled : public std::exception {
public:
    const char* what() const noexcept override { return "已取消生成方案。"; }
};

struct OptimizationResult {
    Grid grid;
    BuildRequest request;
    long long minCoolingMargin = 0;
    int usefulBlocks = 0;
    int disconnectedFunctionalBlocks = 0;
    int functionalIrradiators = 0;
    double irradiatorFlux = 0.0;

    explicit OptimizationResult(Grid initial) : grid(std::move(initial)) {}
    OptimizationResult(Grid initial, BuildRequest buildRequest) : grid(std::move(initial)), request(std::move(buildRequest)) {}
};

int requiredSourceCount(const BuildRequest& request);
OptimizationResult optimizeLayout(const BuildRequest& request, const std::atomic_bool* cancelRequested = nullptr);

} // namespace ncfr
