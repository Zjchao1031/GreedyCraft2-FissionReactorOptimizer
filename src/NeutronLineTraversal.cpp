#include "NeutronLineTraversal.h"

#include "NeutronRules.h"

#include <cmath>

namespace ncfr {
namespace {

NeutronLineResult terminalResult(NeutronLineEndpoint endpoint, int targetIndex,
                                 double flux) {
    return {endpoint, targetIndex, flux};
}

} // namespace

NeutronLineResult traceNeutronLine(const Grid& grid, const Fuel& fuel,
                                   const Pos& from, const Pos& direction,
                                   NeutronLineActivity* activity) {
    Pos next{from.x + direction.x, from.y + direction.y,
             from.z + direction.z};
    if (!grid.inBounds(next.x, next.y, next.z)) {
        return {};
    }

    const Block& adjacent = grid.at(next.x, next.y, next.z);
    if (fuel.intrinsicFlux > 0.0) {
        const int targetIndex = grid.index(next.x, next.y, next.z);
        if (adjacent.kind == BlockKind::FuelCell) {
            return terminalResult(NeutronLineEndpoint::FuelCell, targetIndex,
                                  fuel.intrinsicFlux);
        }
        if (adjacent.kind == BlockKind::Irradiator && adjacent.type >= 0) {
            return terminalResult(NeutronLineEndpoint::Irradiator,
                                  targetIndex, fuel.intrinsicFlux);
        }
    }

    double lineFlux = fuel.intrinsicFlux;
    Pos current = next;
    for (int step = 1; step <= kNeutronReach; ++step) {
        if (!grid.inBounds(current.x, current.y, current.z)) {
            return {};
        }

        const int currentIndex =
            grid.index(current.x, current.y, current.z);
        const Block& block = grid.atIndex(currentIndex);
        if (block.kind == BlockKind::Moderator && block.type >= 0) {
            lineFlux +=
                moderatorTypes().at(static_cast<size_t>(block.type)).fluxFactor;
            if (activity != nullptr) {
                activity->moderatorIndices.push_back(currentIndex);
            }
        } else if (block.kind == BlockKind::Shield && block.type >= 0) {
            if (activity != nullptr) {
                activity->shields.push_back({currentIndex, lineFlux});
            }
        } else {
            return {};
        }

        Pos target{current.x + direction.x, current.y + direction.y,
                   current.z + direction.z};
        if (!grid.inBounds(target.x, target.y, target.z)) {
            return {};
        }

        const int targetIndex = grid.index(target.x, target.y, target.z);
        const Block& targetBlock = grid.atIndex(targetIndex);
        if (targetBlock.kind == BlockKind::FuelCell) {
            return terminalResult(NeutronLineEndpoint::FuelCell, targetIndex,
                                  lineFlux);
        }
        if (targetBlock.kind == BlockKind::Irradiator &&
            targetBlock.type >= 0) {
            return terminalResult(NeutronLineEndpoint::Irradiator,
                                  targetIndex, lineFlux);
        }
        if (targetBlock.kind == BlockKind::Reflector &&
            targetBlock.type >= 0 &&
            step <= kMaxReflectorLineModerators) {
            const double reflectedFlux =
                std::floor(2.0 * lineFlux *
                           reflectorTypes()
                               .at(static_cast<size_t>(targetBlock.type))
                               .reflectivity);
            return terminalResult(NeutronLineEndpoint::Reflector, targetIndex,
                                  reflectedFlux);
        }
        current = target;
    }

    return {};
}

} // namespace ncfr
