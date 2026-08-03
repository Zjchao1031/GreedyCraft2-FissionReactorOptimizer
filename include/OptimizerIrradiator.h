#pragma once

#include "OptimizerTypes.h"

namespace ncfr::optimizer_detail {

OptimizationResult optimizeSixFuelIrradiatorLayout(
    const BuildRequest& request, const std::atomic_bool* cancelRequested);

} // namespace ncfr::optimizer_detail
