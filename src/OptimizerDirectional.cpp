#include "OptimizerDirectional.h"

#include "OptimizerCommon.h"

#include "NeutronRules.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>
namespace ncfr::optimizer_detail {

constexpr double kFluxEpsilon = 1e-9;

bool samePos(const Pos& lhs, const Pos& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool containsDirectionIndex(const std::vector<int>& indices, int index) {
    return std::find(indices.begin(), indices.end(), index) != indices.end();
}

bool isFullyReflectiveReflectorType(int reflectorType) {
    return reflectorType >= 0 &&
           reflectorType < static_cast<int>(reflectorTypes().size()) &&
           reflectorTypes().at(static_cast<size_t>(reflectorType)).reflectivity >= 1.0;
}

double estimatedLineFlux(const Fuel& fuel, int moderatorType, int moderatorCount, int reflectorType) {
    const auto& moderator = moderatorTypes().at(static_cast<size_t>(moderatorType));
    const auto& reflector = reflectorTypes().at(static_cast<size_t>(reflectorType));
    const double lineFlux = fuel.intrinsicFlux + moderator.fluxFactor * moderatorCount;
    return std::floor(2.0 * lineFlux * reflector.reflectivity);
}

bool fuelLineWithinReflectorReach(const FuelLineSpec& line) {
    return line.moderatorCount >= 1 && line.moderatorCount <= kMaxReflectorLineModerators;
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

std::vector<Dimension> singleFuelSearchDimensions() {
    return {{kMaxSize, kMaxSize, kMaxSize}};
}

bool placeDirectionalSources(Grid& grid, const BuildRequest& request, const Pos& fuelPos,
                             const std::vector<int>& sourceDirections) {
    if (static_cast<int>(sourceDirections.size()) != requiredSourceCountForFuels(request)) {
        return false;
    }
    for (int sourceDirection : sourceDirections) {
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(sourceDirection));
        const Pos sourcePos = sourcePositionForDirection(grid, fuelPos, dir);
        if (!grid.isBoundary(sourcePos.x, sourcePos.y, sourcePos.z)) {
            return false;
        }
        Block& block = grid.at(sourcePos.x, sourcePos.y, sourcePos.z);
        if (block.kind != BlockKind::Casing && block.kind != BlockKind::Source) {
            return false;
        }
        block = {BlockKind::Source, -1};
    }
    return allSourcesTargetFuel(grid);
}

void markProtected(StateVector& protectedPositions, const Grid& grid, const Pos& pos) {
    if (grid.inBounds(pos.x, pos.y, pos.z)) {
        protectedPositions.at(static_cast<size_t>(grid.index(pos.x, pos.y, pos.z))) = true;
    }
}

bool isFullyReflectiveReflector(const Block& block) {
    return block.kind == BlockKind::Reflector && block.type >= 0 &&
           reflectorTypes().at(static_cast<size_t>(block.type)).reflectivity >= 1.0;
}

Block sourceLineReplacementBlock(const BuildRequest& request) {
    int bestType = -1;
    double bestReflectivity = -1.0;
    for (int type : request.selectedReflectorTypeIndices) {
        const double reflectivity = reflectorTypes().at(static_cast<size_t>(type)).reflectivity;
        if (reflectivity >= 1.0) {
            continue;
        }
        if (reflectivity > bestReflectivity ||
            (reflectivity == bestReflectivity && (bestType < 0 || type < bestType))) {
            bestType = type;
            bestReflectivity = reflectivity;
        }
    }
    return bestType >= 0 ? Block{BlockKind::Reflector, bestType} : Block{BlockKind::Empty, -1};
}

void keepSourceLinesOpen(Grid& grid, const BuildRequest& request, const std::vector<int>& sourceDirections) {
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != 1) {
        return;
    }

    const Block replacement = sourceLineReplacementBlock(request);
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
                block = replacement;
            }
        }
    }
}

bool restoreDirectionalFuelLines(Grid& grid, const BuildRequest& request, const std::vector<int>& sourceDirections,
                                 const std::vector<FuelLineSpec>& fuelLines) {
    const std::vector<Pos> fuelPositions = fuelPositionsInGrid(grid);
    if (fuelPositions.size() != 1) {
        return false;
    }

    const Pos fuelPos = fuelPositions.front();
    for (const FuelLineSpec& line : fuelLines) {
        if (!fuelLineWithinReflectorReach(line)) {
            return false;
        }
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(line.direction));
        for (int distance = 1; distance <= line.moderatorCount; ++distance) {
            const Pos moderatorPos = offset(fuelPos, dir, distance);
            if (!grid.isInterior(moderatorPos.x, moderatorPos.y, moderatorPos.z)) {
                return false;
            }
            grid.at(moderatorPos.x, moderatorPos.y, moderatorPos.z) = {BlockKind::Moderator, line.moderatorType};
        }

        const Pos reflectorPos = offset(fuelPos, dir, line.moderatorCount + 1);
        if (!grid.isInterior(reflectorPos.x, reflectorPos.y, reflectorPos.z)) {
            return false;
        }
        grid.at(reflectorPos.x, reflectorPos.y, reflectorPos.z) = {BlockKind::Reflector, line.reflectorType};
    }

    keepSourceLinesOpen(grid, request, sourceDirections);
    return placeDirectionalSources(grid, request, fuelPos, sourceDirections);
}


std::vector<FuelLineSpec> singleFuelLineOptions(const Fuel& fuel, const BuildRequest& request,
                                                const std::vector<int>& sourceDirections,
                                                int direction) {
    std::vector<FuelLineSpec> options;
    for (int reflectorType : request.selectedReflectorTypeIndices) {
        if (containsDirectionIndex(sourceDirections, direction) &&
            isFullyReflectiveReflectorType(reflectorType)) {
            continue;
        }
        for (int moderatorType : request.selectedModeratorTypeIndices) {
            for (int moderators = 1; moderators <= kMaxReflectorLineModerators; ++moderators) {
                const double flux = estimatedLineFlux(fuel, moderatorType, moderators, reflectorType);
                if (flux > 2.0 * fuel.criticality + kFluxEpsilon) {
                    continue;
                }
                options.push_back({direction, moderators, moderatorType, reflectorType, flux});
            }
        }
    }
    std::sort(options.begin(), options.end(), [](const FuelLineSpec& lhs, const FuelLineSpec& rhs) {
        if (lhs.moderatorCount != rhs.moderatorCount) {
            return lhs.moderatorCount > rhs.moderatorCount;
        }
        if (lhs.estimatedFlux != rhs.estimatedFlux) {
            return lhs.estimatedFlux < rhs.estimatedFlux;
        }
        if (lhs.moderatorType != rhs.moderatorType) {
            return lhs.moderatorType < rhs.moderatorType;
        }
        if (lhs.reflectorType != rhs.reflectorType) {
            return lhs.reflectorType < rhs.reflectorType;
        }
        return lhs.direction < rhs.direction;
    });
    return options;
}

} // namespace ncfr::optimizer_detail
