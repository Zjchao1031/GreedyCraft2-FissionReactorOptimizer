#include "OptimizerDetail.h"

#include "Perf.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ncfr::optimizer_detail {

bool containsDirectionIndex(const std::vector<int>& indices, int index) {
    return std::find(indices.begin(), indices.end(), index) != indices.end();
}

double estimatedReflectorFluxForType(const Fuel& fuel, int reflectorType) {
    const double lineFlux = fuel.intrinsicFlux + moderatorTypes().at(2).fluxFactor;
    const auto& reflector = reflectorTypes().at(static_cast<size_t>(reflectorType));
    return std::floor(2.0 * lineFlux * reflector.reflectivity);
}

int reflectorTypeForDirection(const Fuel& fuel, const std::vector<int>& sourceDirections, int direction) {
    if (containsDirectionIndex(sourceDirections, direction)) {
        return 1;
    }

    constexpr int kFullReflectorType = 0;
    constexpr int kWeakReflectorType = 1;
    const double fullReflectorFlux = estimatedReflectorFluxForType(fuel, kFullReflectorType);
    if (fullReflectorFlux <= 2.0 * fuel.criticality + 1e-9) {
        return kFullReflectorType;
    }
    return kWeakReflectorType;
}

Pos offset(const Pos& pos, const Direction& dir, int distance) {
    return {pos.x + dir.dx * distance, pos.y + dir.dy * distance, pos.z + dir.dz * distance};
}

Pos sourcePositionForDirection(const Grid& grid, const Pos& fuelPos, const Direction& dir) {
    Pos pos{fuelPos.x, fuelPos.y, fuelPos.z};
    if (dir.dx > 0) pos.x = grid.width() - 1;
    if (dir.dx < 0) pos.x = 0;
    if (dir.dy > 0) pos.y = grid.height() - 1;
    if (dir.dy < 0) pos.y = 0;
    if (dir.dz > 0) pos.z = grid.depth() - 1;
    if (dir.dz < 0) pos.z = 0;
    return pos;
}

void enumerateDirectionCombinations(int start, int remaining, std::vector<int>& current,
                                    std::vector<std::vector<int>>& combinations) {
    if (remaining == 0) {
        combinations.push_back(current);
        return;
    }
    for (int index = start; index <= static_cast<int>(kSourceDirections.size()) - remaining; ++index) {
        current.push_back(index);
        enumerateDirectionCombinations(index + 1, remaining - 1, current, combinations);
        current.pop_back();
    }
}

std::vector<std::vector<int>> sourceDirectionCombinations(int sourceCount) {
    std::vector<std::vector<int>> combinations;
    std::vector<int> current;
    enumerateDirectionCombinations(0, sourceCount, current, combinations);
    return combinations;
}

double estimatedReflectorFlux(const Fuel& fuel, bool sourceDirection) {
    const int reflectorType = sourceDirection ? 1 : 0;
    return estimatedReflectorFluxForType(fuel, reflectorType);
}

double estimatedReflectorFlux(const Fuel& fuel, const std::vector<int>& sourceDirections,
                              const std::vector<int>& reflectorDirections) {
    double flux = 0.0;
    for (int direction : reflectorDirections) {
        flux += estimatedReflectorFluxForType(fuel, reflectorTypeForDirection(fuel, sourceDirections, direction));
    }
    return flux;
}

std::vector<std::vector<int>> reflectorDirectionCombinations(const Fuel& fuel,
                                                             const std::vector<int>& sourceDirections) {
    std::vector<int> pool;
    for (int index = 0; index < static_cast<int>(kSourceDirections.size()); ++index) {
        pool.push_back(index);
    }
    std::vector<std::vector<int>> combinations;

    const int subsetCount = 1 << static_cast<int>(pool.size());
    for (int mask = 0; mask < subsetCount; ++mask) {
        std::vector<int> candidate;
        for (int bit = 0; bit < static_cast<int>(pool.size()); ++bit) {
            if ((mask & (1 << bit)) != 0) {
                candidate.push_back(pool.at(static_cast<size_t>(bit)));
            }
        }
        const double estimatedFlux = estimatedReflectorFlux(fuel, sourceDirections, candidate);
        if (estimatedFlux + 1e-9 >= fuel.criticality &&
            estimatedFlux <= 2.0 * fuel.criticality + 1e-9) {
            combinations.push_back(std::move(candidate));
        }
    }

    std::sort(combinations.begin(), combinations.end(), [&](const auto& lhs, const auto& rhs) {
        if (lhs.size() != rhs.size()) {
            return lhs.size() < rhs.size();
        }
        const double lhsFlux = estimatedReflectorFlux(fuel, sourceDirections, lhs);
        const double rhsFlux = estimatedReflectorFlux(fuel, sourceDirections, rhs);
        if (lhsFlux != rhsFlux) {
            return lhsFlux < rhsFlux;
        }
        return lhs < rhs;
    });
    return combinations;
}

std::vector<Dimension> singleFuelSearchDimensions(const FuelActivationProfile& profile) {
    static constexpr std::array<Dimension, 4> dims = {{
        {5, 5, 5},
        {15, 15, 15},
        {18, 18, 18},
        {24, 24, 24},
    }};

    size_t start = 0;
    if (profile.minSearchInteriorSize >= 18) {
        start = 2;
    } else if (profile.minSearchInteriorSize >= 15) {
        start = 1;
    }

    return std::vector<Dimension>(dims.begin() + static_cast<std::ptrdiff_t>(start), dims.end());
}

bool placeDirectionalSources(Grid& grid, const BuildRequest& request, const Pos& fuelPos,
                             const std::vector<int>& sourceDirections) {
    if (static_cast<int>(sourceDirections.size()) != requiredSourceCountForFuels(request)) {
        return false;
    }
    for (int sourceDirection : sourceDirections) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(sourceDirection));
        const Pos sourcePos = sourcePositionForDirection(grid, fuelPos, dir);
        if (!grid.isBoundary(sourcePos.x, sourcePos.y, sourcePos.z) ||
            grid.at(sourcePos.x, sourcePos.y, sourcePos.z).kind != BlockKind::Casing) {
            return false;
        }
        grid.at(sourcePos.x, sourcePos.y, sourcePos.z) = {BlockKind::Source, -1};
    }
    return allSourcesTargetFuel(grid);
}

std::optional<Grid> buildSingleFuelSkeleton(const Dimension& dim, const BuildRequest& request,
                                            const std::vector<int>& sourceDirections,
                                            const std::vector<int>& reflectorDirections) {
    Grid grid = makeShell(dim.a, dim.b, dim.c);
    const Fuel& fuel = fuels().at(static_cast<size_t>(request.fuelIndices.front()));
    const Pos fuelPos{(dim.a + 1) / 2, (dim.b + 1) / 2, (dim.c + 1) / 2};
    grid.at(fuelPos.x, fuelPos.y, fuelPos.z) = {BlockKind::FuelCell, request.fuelIndices.front()};

    for (int index = 0; index < static_cast<int>(kSourceDirections.size()); ++index) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(index));
        const Pos moderatorPos = offset(fuelPos, dir, 1);
        const Pos reflectorPos = offset(fuelPos, dir, 2);
        if (!grid.isInterior(moderatorPos.x, moderatorPos.y, moderatorPos.z) ||
            !grid.isInterior(reflectorPos.x, reflectorPos.y, reflectorPos.z)) {
            return std::nullopt;
        }
        if (containsDirectionIndex(reflectorDirections, index)) {
            grid.at(moderatorPos.x, moderatorPos.y, moderatorPos.z) = {BlockKind::Moderator, 2};
            grid.at(reflectorPos.x, reflectorPos.y, reflectorPos.z) =
                {BlockKind::Reflector, reflectorTypeForDirection(fuel, sourceDirections, index)};
        }
    }

    if (!placeDirectionalSources(grid, request, fuelPos, sourceDirections)) {
        return std::nullopt;
    }
    return grid;
}

bool isFullyReflectiveReflector(const Block& block) {
    return block.kind == BlockKind::Reflector && block.type >= 0 &&
           reflectorTypes().at(static_cast<size_t>(block.type)).reflectivity >= 1.0;
}

void keepSourceLinesOpen(Grid& grid, const std::vector<int>& sourceDirections) {
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != 1) {
        return;
    }

    for (int sourceDirection : sourceDirections) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(sourceDirection));
        Pos pos = sourcePositionForDirection(grid, fuelPositions.front(), dir);
        while (grid.inBounds(pos.x, pos.y, pos.z)) {
            pos.x -= dir.dx;
            pos.y -= dir.dy;
            pos.z -= dir.dz;
            if (!grid.inBounds(pos.x, pos.y, pos.z) ||
                (pos.x == fuelPositions.front().x && pos.y == fuelPositions.front().y &&
                 pos.z == fuelPositions.front().z)) {
                break;
            }
            Block& block = grid.at(pos.x, pos.y, pos.z);
            if (isFullyReflectiveReflector(block)) {
                block = {BlockKind::Reflector, 1};
            }
        }
    }
}

bool restoreDirectionalFuelLines(Grid& grid, const BuildRequest& request, const std::vector<int>& sourceDirections,
                                 const std::vector<int>& reflectorDirections) {
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != 1) {
        return false;
    }

    const Pos fuelPos = fuelPositions.front();
    const Fuel& fuel = fuels().at(static_cast<size_t>(request.fuelIndices.front()));
    for (int reflectorDirection : reflectorDirections) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(reflectorDirection));
        const Pos moderatorPos = offset(fuelPos, dir, 1);
        const Pos reflectorPos = offset(fuelPos, dir, 2);
        if (!grid.isInterior(moderatorPos.x, moderatorPos.y, moderatorPos.z) ||
            !grid.isInterior(reflectorPos.x, reflectorPos.y, reflectorPos.z)) {
            return false;
        }
        grid.at(moderatorPos.x, moderatorPos.y, moderatorPos.z) = {BlockKind::Moderator, 2};
        grid.at(reflectorPos.x, reflectorPos.y, reflectorPos.z) =
            {BlockKind::Reflector, reflectorTypeForDirection(fuel, sourceDirections, reflectorDirection)};
    }

    keepSourceLinesOpen(grid, sourceDirections);
    return true;
}

void pruneInactiveSupport(Grid& grid) {
    FuelSimulation sim = simulateMixedFuel(grid);
    for (const Pos& pos : grid.interiorPositions()) {
        const int idx = grid.index(pos.x, pos.y, pos.z);
        Block& block = grid.atIndex(idx);
        if (block.kind != BlockKind::Empty && isSupportMutable(block) &&
            !isRequiredSupportBlock(grid, sim, idx)) {
            block = {BlockKind::Empty, -1};
        }
    }
}

std::optional<Grid> compactInteriorPlanesPreservingSources(const Grid& grid, const BuildRequest& request,
                                                           const std::vector<int>& sourceDirections,
                                                           int paddingPlanes = 0) {
    NCFR_PERF_COUNT(compactInteriorPlanesCalls);
    NCFR_PERF_SCOPE(compactInteriorPlanesNs);
    std::vector<bool> keepX(static_cast<size_t>(grid.internalA() + 1), false);
    std::vector<bool> keepY(static_cast<size_t>(grid.internalB() + 1), false);
    std::vector<bool> keepZ(static_cast<size_t>(grid.internalC() + 1), false);

    auto keepRange = [paddingPlanes](std::vector<bool>& keep, int center, int max) {
        const int begin = std::max(1, center - paddingPlanes);
        const int end = std::min(max, center + paddingPlanes);
        for (int value = begin; value <= end; ++value) {
            keep.at(static_cast<size_t>(value)) = true;
        }
    };

    for (const Pos& pos : grid.interiorPositions()) {
        if (grid.at(pos.x, pos.y, pos.z).kind == BlockKind::Empty) {
            continue;
        }
        keepRange(keepX, pos.x, grid.internalA());
        keepRange(keepY, pos.y, grid.internalB());
        keepRange(keepZ, pos.z, grid.internalC());
    }

    const int newA = static_cast<int>(std::count(keepX.begin(), keepX.end(), true));
    const int newB = static_cast<int>(std::count(keepY.begin(), keepY.end(), true));
    const int newC = static_cast<int>(std::count(keepZ.begin(), keepZ.end(), true));
    if (newA <= 0 || newB <= 0 || newC <= 0) {
        return std::nullopt;
    }

    std::vector<int> mapX(static_cast<size_t>(grid.internalA() + 1), 0);
    std::vector<int> mapY(static_cast<size_t>(grid.internalB() + 1), 0);
    std::vector<int> mapZ(static_cast<size_t>(grid.internalC() + 1), 0);
    for (int x = 1, next = 1; x <= grid.internalA(); ++x) {
        if (keepX.at(static_cast<size_t>(x))) {
            mapX.at(static_cast<size_t>(x)) = next++;
        }
    }
    for (int y = 1, next = 1; y <= grid.internalB(); ++y) {
        if (keepY.at(static_cast<size_t>(y))) {
            mapY.at(static_cast<size_t>(y)) = next++;
        }
    }
    for (int z = 1, next = 1; z <= grid.internalC(); ++z) {
        if (keepZ.at(static_cast<size_t>(z))) {
            mapZ.at(static_cast<size_t>(z)) = next++;
        }
    }

    Grid compacted = makeShell(newA, newB, newC);
    for (const Pos& pos : grid.interiorPositions()) {
        const Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::Empty) {
            continue;
        }
        compacted.at(mapX.at(static_cast<size_t>(pos.x)),
                     mapY.at(static_cast<size_t>(pos.y)),
                     mapZ.at(static_cast<size_t>(pos.z))) = block;
    }

    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(compacted);
    if (fuelPositions.size() != request.fuelIndices.size() ||
        !placeDirectionalSources(compacted, request, fuelPositions.front(), sourceDirections)) {
        return std::nullopt;
    }
    return compacted;
}

FinalizeResult tryFinalizeDirectionalCandidate(Grid grid, const BuildRequest& request,
                                               const std::vector<int>& sourceDirections,
                                               const std::vector<int>& reflectorDirections,
                                               const std::atomic_bool* cancelRequested) {
    NCFR_PERF_COUNT(finalizeCandidateCalls);
    NCFR_PERF_SCOPE(finalizeCandidateNs);
    if (!restoreDirectionalFuelLines(grid, request, sourceDirections, reflectorDirections)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("restoreFuelLinesFailed", grid, nullptr, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, FinalizeFailureKind::Structural};
    }
    const FuelRelationPrefilterResult relation = prefilterFuelRelations(grid, request);
    if (!relation.accepted) {
#ifndef NDEBUG
        std::ostringstream detail;
        detail << directionalGridDetail("fuelRelationRejected", grid, nullptr, request, sourceDirections,
                                        reflectorDirections)
               << " " << fuelRelationDetail("prefilter", relation, request);
        const std::string checkpoint = detail.str();
        logFinalizeCheckpoint("finalize.reject", checkpoint, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, finalizeFailureFromFuelRelation(relation)};
    }
    pruneInactiveSupport(grid);
    FuelSimulation preCompactSim = simulateMixedFuel(grid);
    if (!isPreCompactRunnable(preCompactSim)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("preCompactNotRunnable", grid, &preCompactSim, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, classifyFinalizationFailure(grid, preCompactSim, request)};
    }

    std::optional<Grid> compacted =
        compactInteriorPlanesPreservingSources(grid, request, sourceDirections);
    const bool compactedHasNoEmptyPlane = compacted.has_value() && hasNoEmptyInteriorPlane(*compacted);
    if (!compacted.has_value() || !compactedHasNoEmptyPlane) {
#ifndef NDEBUG
        const char* reason =
            !compacted.has_value() ? "compactPreservingSourcesFailed" : "emptyInteriorPlaneAfterCompact";
        const Grid& detailGrid = compacted.has_value() ? *compacted : grid;
        const std::string detail = directionalGridDetail(reason, detailGrid, nullptr, request, sourceDirections,
                                                         reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, FinalizeFailureKind::Structural};
    }

    fillSupportBlocks(*compacted);
    Grid improved = improveSupportBlocks(std::move(*compacted), cancelRequested);
    if (!restoreDirectionalFuelLines(improved, request, sourceDirections, reflectorDirections)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("postCompactRestoreFuelLinesFailed", improved, nullptr,
                                                         request, sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, FinalizeFailureKind::Structural};
    }
    pruneInactiveSupport(improved);
    FuelSimulation sim = simulateMixedFuel(improved);
    if (classifyFinalizationFailure(improved, sim, request) == FinalizeFailureKind::CoolingDeficit) {
        improved = expandCooling(std::move(improved), request, sourceDirections, reflectorDirections,
                                 cancelRequested, kCoolingExpansionOptions);
        if (!restoreDirectionalFuelLines(improved, request, sourceDirections, reflectorDirections)) {
#ifndef NDEBUG
            const std::string detail = directionalGridDetail("postCoolingExpandRestoreFuelLinesFailed", improved,
                                                             nullptr, request, sourceDirections, reflectorDirections);
            logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
            return {std::nullopt, FinalizeFailureKind::Structural};
        }
        pruneInactiveSupport(improved);
        sim = simulateMixedFuel(improved);
    }
    if (!isAccepted(improved, sim)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("improvedNotAccepted", improved, &sim, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, classifyFinalizationFailure(improved, sim, request)};
    }

    std::optional<Grid> finalCompacted = compactInteriorPlanesPreservingSources(improved, request, sourceDirections);
    bool finalRestored = false;
    bool finalHasNoEmptyPlane = false;
    if (finalCompacted.has_value()) {
        finalRestored = restoreDirectionalFuelLines(*finalCompacted, request, sourceDirections, reflectorDirections);
        if (finalRestored) {
            finalHasNoEmptyPlane = hasNoEmptyInteriorPlane(*finalCompacted);
        }
    }
    if (!finalCompacted.has_value() || !finalRestored || !finalHasNoEmptyPlane) {
#ifndef NDEBUG
        const char* reason = "finalCompactValidationFailed";
        if (!finalCompacted.has_value()) {
            reason = "finalCompactPreservingSourcesFailed";
        } else if (!finalRestored) {
            reason = "finalRestoreFuelLinesFailed";
        } else if (!finalHasNoEmptyPlane) {
            reason = "finalEmptyInteriorPlane";
        }
        const Grid& detailGrid = finalCompacted.has_value() ? *finalCompacted : improved;
        const std::string detail = directionalGridDetail(reason, detailGrid, nullptr, request, sourceDirections,
                                                         reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, FinalizeFailureKind::Structural};
    }
    FuelSimulation finalSim = simulateMixedFuel(*finalCompacted);
    if (!isAccepted(*finalCompacted, finalSim)) {
#ifndef NDEBUG
        const std::string detail = directionalGridDetail("finalNotAccepted", *finalCompacted, &finalSim, request,
                                                         sourceDirections, reflectorDirections);
        logFinalizeCheckpoint("finalize.reject", detail, 0, kDefaultImproveOptions);
#endif
        return {std::nullopt, classifyFinalizationFailure(*finalCompacted, finalSim, request)};
    }
#ifndef NDEBUG
    const std::string detail = directionalGridDetail("accepted", *finalCompacted, &finalSim, request,
                                                     sourceDirections, reflectorDirections);
    logFinalizeCheckpoint("finalize.accept", detail, 0, kDefaultImproveOptions);
#endif
    return {resultFromSimulation(std::move(*finalCompacted), request, finalSim), FinalizeFailureKind::None};
}

class OptimizationStrategy {
public:
    virtual ~OptimizationStrategy() = default;
    virtual OptimizationResult optimize(const BuildRequest& request, const std::atomic_bool* cancelRequested) const = 0;
};

OptimizationResult optimizeSingleFuelDirectionalLayout(const BuildRequest& request,
                                                       const std::vector<std::vector<int>>& sourceCombos,
                                                       const std::atomic_bool* cancelRequested) {
    const Fuel& fuel = fuels().at(static_cast<size_t>(request.fuelIndices.front()));
    const FuelActivationProfile& profile = fuelActivationProfile(request.fuelIndices.front());
    const std::vector<Dimension> dims = singleFuelSearchDimensions(profile);

    for (const Dimension& dim : dims) {
        for (const std::vector<int>& sourceDirections : sourceCombos) {
            for (const std::vector<int>& reflectorDirections : reflectorDirectionCombinations(fuel, sourceDirections)) {
                throwIfCancelled(cancelRequested);
#ifndef NDEBUG
                {
                    const std::string detail =
                        directionalCandidateDetail("start", dim, sourceDirections, reflectorDirections);
                    NCFR_PERF_CHECKPOINT("candidate.directional", detail.c_str());
                }
#endif
                std::optional<Grid> candidate;
                {
                    NCFR_PERF_SCOPE(candidateGenerationNs);
                    candidate = buildSingleFuelSkeleton(dim, request, sourceDirections, reflectorDirections);
                }
                if (!candidate.has_value()) {
#ifndef NDEBUG
                    const std::string detail =
                        directionalCandidateDetail("skeletonRejected", dim, sourceDirections, reflectorDirections);
                    NCFR_PERF_CHECKPOINT("candidate.directional", detail.c_str());
#endif
                    continue;
                }
                const FuelRelationPrefilterResult relation = prefilterFuelRelations(*candidate, request);
                if (!relation.accepted) {
#ifndef NDEBUG
                    std::ostringstream detail;
                    detail << directionalCandidateDetail("fuelRelationRejected", dim, sourceDirections,
                                                         reflectorDirections)
                           << " " << fuelRelationDetail("prefilter", relation, request);
                    const std::string checkpoint = detail.str();
                    NCFR_PERF_CHECKPOINT("candidate.directional", checkpoint.c_str());
#endif
                    continue;
                }
                NCFR_PERF_COUNT(candidateCount);
                NCFR_PERF_COUNT(candidateEvaluations);
                NCFR_PERF_SCOPE(candidateEvaluationNs);
                fillSupportBlocks(*candidate);
                Grid improved = improveSupportBlocks(std::move(*candidate), cancelRequested);
                if (!restoreDirectionalFuelLines(improved, request, sourceDirections, reflectorDirections)) {
#ifndef NDEBUG
                    const std::string detail = directionalGridDetail("postImproveRestoreFuelLinesFailed", improved,
                                                                     nullptr, request, sourceDirections,
                                                                     reflectorDirections);
                    NCFR_PERF_CHECKPOINT("candidate.reject", detail.c_str());
#endif
                    continue;
                }
                throwIfCancelled(cancelRequested);
                FinalizeResult result = tryFinalizeDirectionalCandidate(
                    std::move(improved), request, sourceDirections, reflectorDirections, cancelRequested);
                if (result.result.has_value()) {
                    NCFR_PERF_COUNT(bestUpdates);
                    return std::move(*result.result);
                }
            }
        }
    }

    throw std::runtime_error("无满足输入要求的搭建方法。");
}

BuildRequest singleFuelRequestForSlot(const BuildRequest& request, int slot) {
    BuildRequest single;
    single.fuelIndices = {request.fuelIndices.at(static_cast<size_t>(slot))};
    return single;
}

OptimizationResult optimizeSingleFuelForSlot(const BuildRequest& request, int slot,
                                             const std::atomic_bool* cancelRequested) {
    BuildRequest single = singleFuelRequestForSlot(request, slot);
    const Fuel& fuel = fuels().at(static_cast<size_t>(single.fuelIndices.front()));
    if (fuel.selfPriming) {
        return optimizeSingleFuelDirectionalLayout(single, {{}}, cancelRequested);
    }

    const std::vector<std::vector<int>> sourceCombos = sourceDirectionCombinations(requiredSourceCountForFuels(single));
    return optimizeSingleFuelDirectionalLayout(single, sourceCombos, cancelRequested);
}

bool heatPriorityLess(int lhsSlot, int rhsSlot, const BuildRequest& request) {
    const Fuel& lhs = fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(lhsSlot))));
    const Fuel& rhs = fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(rhsSlot))));
    if (lhs.heat != rhs.heat) {
        return lhs.heat > rhs.heat;
    }
    if (lhs.criticality != rhs.criticality) {
        return lhs.criticality > rhs.criticality;
    }
    return lhsSlot < rhsSlot;
}

int primarySameStartupSlot(const BuildRequest& request) {
    return heatPriorityLess(0, 1, request) ? 0 : 1;
}

} // namespace ncfr::optimizer_detail
