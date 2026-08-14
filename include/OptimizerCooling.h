#pragma once

#include "OptimizerTypes.h"

#include <functional>

namespace ncfr::optimizer_detail {

Grid expandCooling(Grid grid, const BuildRequest& request,
                   const std::vector<int>& sourceDirections,
                   const std::vector<FuelLineSpec>& fuelLines,
                   const std::atomic_bool* cancelRequested,
                   const CoolingExpansionOptions& options);
Grid expandCoolingWithPreserver(
    Grid grid, const std::function<bool(Grid&)>& preserveGrid,
    const std::atomic_bool* cancelRequested,
    const CoolingExpansionOptions& options,
    bool allowDisconnectedFunctionalBlocks = false,
    CoolingValidationPolicy coolingPolicy =
        CoolingValidationPolicy::PerCluster);

} // namespace ncfr::optimizer_detail
