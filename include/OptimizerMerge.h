#pragma once

#include "OptimizerTypes.h"

namespace ncfr::optimizer_detail {

OptimizationResult optimizeDualFuelLayout(const BuildRequest& request,
                                          const std::atomic_bool* cancelRequested);
OptimizationResult optimizeQuadFuelLayout(const BuildRequest& request,
                                          const std::atomic_bool* cancelRequested);

} // namespace ncfr::optimizer_detail
