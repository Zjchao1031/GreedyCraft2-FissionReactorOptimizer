#include "Output.h"

#include "Grid.h"

#include <ostream>

namespace ncfr {

void printResult(std::ostream& os, const OptimizationResult& result) {
    os << "燃料单元数量: " << result.request.fuelIndices.size() << '\n';
    os << "中子源数量: " << requiredSourceCount(result.request) << '\n';
    os << "最小冷却余量: " << result.minCoolingMargin << " H/t\n";
    os << "断开功能块: " << result.disconnectedFunctionalBlocks << '\n';
    os << "有效辐照仓: " << result.functionalIrradiators << '\n';
    os << "辐照仓总通量: " << result.irradiatorFlux << '\n';
    os << "输出层数: " << result.grid.depth() << '\n';

    for (int z = 0; z < result.grid.depth(); ++z) {
        os << "第 " << (z + 1) << " 层 (z=" << z << "):\n";
        for (int y = 0; y < result.grid.height(); ++y) {
            for (int x = 0; x < result.grid.width(); ++x) {
                if (x > 0) {
                    os << '\t';
                }
                os << blockDisplayName(result.grid.at(x, y, z));
            }
            os << '\n';
        }
    }
}

} // namespace ncfr
