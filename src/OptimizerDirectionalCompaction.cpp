#include "OptimizerDirectional.h"

#include "OptimizerCommon.h"
#include "OptimizerDiagnostics.h"
#include "Perf.h"

#include <algorithm>
#include <optional>
#include <sstream>
#include <vector>
namespace ncfr::optimizer_detail {

struct DirectionalCompactionPlan {
    std::vector<bool> keepX;
    std::vector<bool> keepY;
    std::vector<bool> keepZ;
    std::vector<int> mapX;
    std::vector<int> mapY;
    std::vector<int> mapZ;
    int newA = 0;
    int newB = 0;
    int newC = 0;
    bool keepConductors = false;
};

#ifndef NDEBUG
std::string keptPlaneRangeLabel(const std::vector<bool>& keep) {
    int first = -1;
    int last = -1;
    int count = 0;
    for (int coordinate = 1; coordinate < static_cast<int>(keep.size()); ++coordinate) {
        if (!keep.at(static_cast<size_t>(coordinate))) {
            continue;
        }
        if (first < 0) {
            first = coordinate;
        }
        last = coordinate;
        ++count;
    }
    if (first < 0) {
        return "none";
    }
    std::ostringstream os;
    os << first << ".." << last << "/" << count;
    return os.str();
}
#endif

std::optional<DirectionalCompactionPlan> buildDirectionalCompactionPlan(
    const Grid& grid, int paddingPlanes, bool keepConductors) {
    DirectionalCompactionPlan plan;
    plan.keepX.assign(static_cast<size_t>(grid.internalA() + 1), false);
    plan.keepY.assign(static_cast<size_t>(grid.internalB() + 1), false);
    plan.keepZ.assign(static_cast<size_t>(grid.internalC() + 1), false);
    plan.keepConductors = keepConductors;

    auto keepRange = [paddingPlanes](std::vector<bool>& keep, int center, int max) {
        const int begin = std::max(1, center - paddingPlanes);
        const int end = std::min(max, center + paddingPlanes);
        for (int value = begin; value <= end; ++value) {
            keep.at(static_cast<size_t>(value)) = true;
        }
    };

    for (const Pos& pos : grid.interiorPositions()) {
        const BlockKind kind = grid.at(pos.x, pos.y, pos.z).kind;
        if (kind == BlockKind::Empty || (!keepConductors && kind == BlockKind::Conductor)) {
            continue;
        }
        keepRange(plan.keepX, pos.x, grid.internalA());
        keepRange(plan.keepY, pos.y, grid.internalB());
        keepRange(plan.keepZ, pos.z, grid.internalC());
    }

    plan.newA = static_cast<int>(std::count(plan.keepX.begin(), plan.keepX.end(), true));
    plan.newB = static_cast<int>(std::count(plan.keepY.begin(), plan.keepY.end(), true));
    plan.newC = static_cast<int>(std::count(plan.keepZ.begin(), plan.keepZ.end(), true));
    if (plan.newA <= 0 || plan.newB <= 0 || plan.newC <= 0) {
        return std::nullopt;
    }

    plan.mapX.assign(static_cast<size_t>(grid.internalA() + 1), 0);
    plan.mapY.assign(static_cast<size_t>(grid.internalB() + 1), 0);
    plan.mapZ.assign(static_cast<size_t>(grid.internalC() + 1), 0);
    for (int x = 1, next = 1; x <= grid.internalA(); ++x) {
        if (plan.keepX.at(static_cast<size_t>(x))) {
            plan.mapX.at(static_cast<size_t>(x)) = next++;
        }
    }
    for (int y = 1, next = 1; y <= grid.internalB(); ++y) {
        if (plan.keepY.at(static_cast<size_t>(y))) {
            plan.mapY.at(static_cast<size_t>(y)) = next++;
        }
    }
    for (int z = 1, next = 1; z <= grid.internalC(); ++z) {
        if (plan.keepZ.at(static_cast<size_t>(z))) {
            plan.mapZ.at(static_cast<size_t>(z)) = next++;
        }
    }
    return plan;
}

std::optional<Grid> applyDirectionalCompactionPlan(
    const Grid& grid, const DirectionalCompactionPlan& plan,
    const BuildRequest& request, const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines) {
    if (plan.keepX.size() != static_cast<size_t>(grid.internalA() + 1) ||
        plan.keepY.size() != static_cast<size_t>(grid.internalB() + 1) ||
        plan.keepZ.size() != static_cast<size_t>(grid.internalC() + 1)) {
        return std::nullopt;
    }

    Grid compacted = makeShell(plan.newA, plan.newB, plan.newC);
    for (const Pos& pos : grid.interiorPositions()) {
        const Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::Empty) {
            continue;
        }
        if (!plan.keepX.at(static_cast<size_t>(pos.x)) ||
            !plan.keepY.at(static_cast<size_t>(pos.y)) ||
            !plan.keepZ.at(static_cast<size_t>(pos.z))) {
            if (!plan.keepConductors && block.kind == BlockKind::Conductor) {
                continue;
            }
            continue;
        }
        compacted.at(plan.mapX.at(static_cast<size_t>(pos.x)),
                     plan.mapY.at(static_cast<size_t>(pos.y)),
                     plan.mapZ.at(static_cast<size_t>(pos.z))) = block;
    }

    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(compacted);
    if (fuelPositions.size() != request.fuelIndices.size() ||
        !restoreDirectionalFuelLines(compacted, request, sourceDirections, fuelLines)) {
        return std::nullopt;
    }
    return compacted;
}

std::optional<Grid> compactInteriorPlanesPreservingSources(const Grid& grid, const BuildRequest& request,
                                                             const std::vector<int>& sourceDirections,
                                                             const std::vector<FuelLineSpec>& fuelLines,
                                                             int paddingPlanes,
                                                             bool keepConductors) {
    NCFR_PERF_COUNT(compactInteriorPlanesCalls);
    NCFR_PERF_SCOPE(compactInteriorPlanesNs);
    const std::optional<DirectionalCompactionPlan> plan =
        buildDirectionalCompactionPlan(grid, paddingPlanes, keepConductors);
    if (!plan.has_value()) {
        return std::nullopt;
    }
#ifndef NDEBUG
    {
        std::ostringstream os;
        os << "old=" << gridInteriorLabel(grid)
           << " new=" << plan->newA << "x" << plan->newB << "x" << plan->newC
           << " keepX=" << keptPlaneRangeLabel(plan->keepX)
           << " keepY=" << keptPlaneRangeLabel(plan->keepY)
           << " keepZ=" << keptPlaneRangeLabel(plan->keepZ)
           << " padding=" << paddingPlanes
           << " keepConductors=" << (keepConductors ? 1 : 0);
        NCFR_PERF_CHECKPOINT("compaction.plan", os.str().c_str());
    }
#endif
    return applyDirectionalCompactionPlan(grid, *plan, request, sourceDirections, fuelLines);
}

std::optional<Grid> compactInteriorPlanesPreservingSourceTargets(
    const Grid& grid, int paddingPlanes, bool keepConductors) {
    NCFR_PERF_COUNT(compactInteriorPlanesCalls);
    NCFR_PERF_SCOPE(compactInteriorPlanesNs);
    const std::optional<DirectionalCompactionPlan> plan =
        buildDirectionalCompactionPlan(
            grid, paddingPlanes, keepConductors);
    if (!plan.has_value()) {
        return std::nullopt;
    }

    Grid compacted = makeShell(
        plan->newA, plan->newB, plan->newC);
    for (const Pos& pos : grid.interiorPositions()) {
        const Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::Empty) {
            continue;
        }
        if (!plan->keepX.at(static_cast<size_t>(pos.x)) ||
            !plan->keepY.at(static_cast<size_t>(pos.y)) ||
            !plan->keepZ.at(static_cast<size_t>(pos.z))) {
            if (!plan->keepConductors &&
                block.kind == BlockKind::Conductor) {
                continue;
            }
            return std::nullopt;
        }
        compacted.at(
            plan->mapX.at(static_cast<size_t>(pos.x)),
            plan->mapY.at(static_cast<size_t>(pos.y)),
            plan->mapZ.at(static_cast<size_t>(pos.z))) = block;
    }

    const auto mapCoordinate =
        [](int value, int oldInternalSize, int newInternalSize,
           const std::vector<bool>& keep,
           const std::vector<int>& map) -> std::optional<int> {
        if (value == 0) {
            return 0;
        }
        if (value == oldInternalSize + 1) {
            return newInternalSize + 1;
        }
        if (value >= 1 && value <= oldInternalSize &&
            keep.at(static_cast<size_t>(value))) {
            return map.at(static_cast<size_t>(value));
        }
        return std::nullopt;
    };
    const auto positionAtIndex = [](const Grid& source,
                                    int index) -> std::optional<Pos> {
        if (index < 0 || index >= source.volume()) {
            return std::nullopt;
        }
        return Pos{
            index % source.width(),
            (index / source.width()) % source.height(),
            index / (source.width() * source.height()),
        };
    };
    const auto mapPosition =
        [&](const Pos& pos) -> std::optional<Pos> {
        const std::optional<int> x = mapCoordinate(
            pos.x, grid.internalA(), compacted.internalA(),
            plan->keepX, plan->mapX);
        const std::optional<int> y = mapCoordinate(
            pos.y, grid.internalB(), compacted.internalB(),
            plan->keepY, plan->mapY);
        const std::optional<int> z = mapCoordinate(
            pos.z, grid.internalC(), compacted.internalC(),
            plan->keepZ, plan->mapZ);
        if (!x.has_value() || !y.has_value() || !z.has_value()) {
            return std::nullopt;
        }
        return Pos{*x, *y, *z};
    };

    std::vector<SourcePrimingTarget> expectedTargets;
    for (const SourcePrimingTarget& target :
         sourcePrimingTargets(grid)) {
        const std::optional<Pos> sourcePos =
            mapPosition(target.source);
        const std::optional<Pos> oldTargetPos =
            positionAtIndex(grid, target.targetIndex);
        if (!sourcePos.has_value() || !oldTargetPos.has_value()) {
            return std::nullopt;
        }
        const std::optional<Pos> targetPos =
            mapPosition(*oldTargetPos);
        if (!targetPos.has_value() ||
            !compacted.isInterior(
                targetPos->x, targetPos->y, targetPos->z)) {
            return std::nullopt;
        }
        compacted.at(
            sourcePos->x, sourcePos->y, sourcePos->z) = {
            BlockKind::Source, -1};
        expectedTargets.push_back({
            *sourcePos,
            compacted.index(
                targetPos->x, targetPos->y, targetPos->z),
        });
    }
    if (!matchesSourcePrimingTargets(compacted, expectedTargets)) {
        return std::nullopt;
    }
    return compacted;
}

} // namespace ncfr::optimizer_detail
