#pragma once

#include "Optimizer.h"

#include <iosfwd>

namespace ncfr {

void printResult(std::ostream& os, const OptimizationResult& result);

} // namespace ncfr
