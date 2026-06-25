#include "OptimizerDetail.h"

#include "NeutronRules.h"
#include "Perf.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ncfr::optimizer_detail {
namespace {

constexpr int kIrradiatorInteriorSize = 17;
constexpr int kIrradiatorCenter = 9;
constexpr int kIrradiatorFuelInputCount = 5;
constexpr int kIrradiatorFuelDistance = kMaxIrradiatorLineModerators + 1;
constexpr int kAnySource = -1;
constexpr double kActivationFluxEpsilon = 1e-9;
constexpr ImproveOptions kIrradiatorImproveOptions{1, 1, 64};
constexpr CoolingExpansionOptions kIrradiatorCoolingExpansionOptions{12, 2, 192, 3, 192, 64, 64, 3};
constexpr Direction kIrradiatorWallBridgeDirection{0, 0, -1};

struct FixedBlock {
    Pos pos;
    Block block;
};

struct ActivationLine {
    int directionIndex = -1;
    std::vector<int> moderatorTypes;
    int reflectorType = -1;
    double reflectedFlux = 0.0;
};

struct ActivationPlan {
    std::vector<ActivationLine> lines;
    double reflectedFlux = 0.0;
};

struct ActivationSearchContext {
    const Fuel* fuel = nullptr;
    std::vector<std::vector<ActivationLine>> optionsByDirection;
    std::vector<double> remainingMaxFlux;
    const std::atomic_bool* cancelRequested = nullptr;
};

struct FixedIrradiatorSkeleton {
    std::vector<FixedBlock> fixedInterior;
    std::vector<Pos> fuelPositions;
    std::vector<ActivationPlan> activations;
    std::vector<Pos> sourcePositions;
    std::vector<Pos> sourceTargets;
    int sourceLineFallbackReflectorType = -1;
};

int oppositeDirectionIndex(int directionIndex) {
    return directionIndex % 2 == 0 ? directionIndex + 1 : directionIndex - 1;
}

int wallConnectionPerpendicularDirectionIndex(int directionIndex) {
    switch (directionIndex) {
    case 0:
    case 1:
        return 2;
    case 2:
    case 3:
        return 4;
    default:
        return 0;
    }
}

std::string fuelCodeOrName(int fuelIndex) {
    const Fuel& fuel = fuels().at(static_cast<size_t>(fuelIndex));
    return fuel.code.empty() ? fuel.nameZh : fuel.code;
}

int strongestSelectedModeratorType(const BuildRequest& request) {
    int bestType = -1;
    int bestFlux = -1;
    for (int type : request.selectedModeratorTypeIndices) {
        const int flux = moderatorTypes().at(static_cast<size_t>(type)).fluxFactor;
        if (flux > bestFlux || (flux == bestFlux && (bestType < 0 || type < bestType))) {
            bestType = type;
            bestFlux = flux;
        }
    }
    if (bestType < 0) {
        throw std::invalid_argument("辐照结构生成需要至少选择一个减速剂。");
    }
    return bestType;
}

int bestWeakSelectedReflectorType(const BuildRequest& request) {
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
    return bestType;
}

bool sameBlock(const Block& lhs, const Block& rhs) {
    return lhs.kind == rhs.kind && lhs.type == rhs.type;
}

bool samePosition(const Pos& lhs, const Pos& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool sameDirection(const Direction& lhs, const Direction& rhs) {
    return lhs.dx == rhs.dx && lhs.dy == rhs.dy && lhs.dz == rhs.dz;
}

bool isIrradiatorWallBridgeDirection(int directionIndex) {
    return sameDirection(kSourceDirections.at(static_cast<size_t>(directionIndex)),
                         kIrradiatorWallBridgeDirection);
}

bool isFixedInteriorPosition(const FixedIrradiatorSkeleton& skeleton, const Pos& pos) {
    return std::any_of(skeleton.fixedInterior.begin(), skeleton.fixedInterior.end(),
                       [&](const FixedBlock& fixed) {
                           return samePosition(fixed.pos, pos);
                       });
}

bool sourceScanDirectionForPosition(const Grid& grid, const Pos& pos, int& axis, int& direction) {
    int matchingFaces = 0;
    auto setDirection = [&](int candidateAxis, int candidateDirection) {
        axis = candidateAxis;
        direction = candidateDirection;
        ++matchingFaces;
    };

    if (pos.x == 0) {
        setDirection(0, 1);
    } else if (pos.x == grid.width() - 1) {
        setDirection(0, -1);
    }
    if (pos.y == 0) {
        setDirection(1, 1);
    } else if (pos.y == grid.height() - 1) {
        setDirection(1, -1);
    }
    if (pos.z == 0) {
        setDirection(2, 1);
    } else if (pos.z == grid.depth() - 1) {
        setDirection(2, -1);
    }
    return matchingFaces == 1;
}

bool keepSourceLineOpen(Grid& grid, const FixedIrradiatorSkeleton& skeleton,
                        const Pos& sourcePos, const Pos& targetPos) {
    int axis = -1;
    int direction = 0;
    if (!sourceScanDirectionForPosition(grid, sourcePos, axis, direction)) {
        return false;
    }

    Pos pos = sourcePos;
    for (int step = 1; step <= kMaxSize; ++step) {
        if (axis == 0) pos.x += direction;
        if (axis == 1) pos.y += direction;
        if (axis == 2) pos.z += direction;
        if (!grid.inBounds(pos.x, pos.y, pos.z)) {
            return false;
        }
        if (samePosition(pos, targetPos)) {
            return true;
        }

        Block& block = grid.at(pos.x, pos.y, pos.z);
        if (isFullyReflectiveReflector(block)) {
            if (isFixedInteriorPosition(skeleton, pos)) {
                return false;
            }
            if (skeleton.sourceLineFallbackReflectorType >= 0) {
                block = {BlockKind::Reflector, skeleton.sourceLineFallbackReflectorType};
            } else {
                block = {BlockKind::Empty, -1};
            }
        }
    }
    return false;
}

bool addFixedBlock(FixedIrradiatorSkeleton& skeleton, const Pos& pos, const Block& block) {
    for (const FixedBlock& fixed : skeleton.fixedInterior) {
        if (samePos(fixed.pos, pos)) {
            return sameBlock(fixed.block, block);
        }
    }
    skeleton.fixedInterior.push_back({pos, block});
    return true;
}

void addWallConnectionChain(FixedIrradiatorSkeleton& skeleton, const Pos& fuelPos, const Direction& outwardDir,
                            int directionIndex) {
    const Direction& perpendicular =
        kSourceDirections.at(static_cast<size_t>(wallConnectionPerpendicularDirectionIndex(directionIndex)));
    const std::array<std::pair<Pos, Block>, 4> chain = {{
        {offset(fuelPos, perpendicular, 1), {BlockKind::Conductor, -1}},
        {offset(offset(fuelPos, outwardDir, 1), perpendicular, 1), {BlockKind::Conductor, -1}},
        {offset(offset(fuelPos, outwardDir, 2), perpendicular, 1), {BlockKind::Conductor, -1}},
        {offset(offset(fuelPos, outwardDir, 3), perpendicular, 1), {BlockKind::Conductor, -1}},
    }};

    for (const auto& [pos, block] : chain) {
        if (!addFixedBlock(skeleton, pos, block)) {
            throw std::runtime_error("燃料到外壳的固定导体连接链发生方块冲突。");
        }
    }
}

double reflectedFluxForActivationLine(const Fuel& fuel, const std::vector<int>& moderatorTypeIndices,
                                      int reflectorType) {
    const auto& reflector = reflectorTypes().at(static_cast<size_t>(reflectorType));
    double lineFlux = fuel.intrinsicFlux;
    for (int moderatorType : moderatorTypeIndices) {
        lineFlux += moderatorTypes().at(static_cast<size_t>(moderatorType)).fluxFactor;
    }
    return std::floor(2.0 * lineFlux * reflector.reflectivity);
}

std::vector<int> activationDirectionsForFuel(int fuelDirectionIndex) {
    std::vector<int> directions;
    const int centerDirection = oppositeDirectionIndex(fuelDirectionIndex);
    const int coolingChainDirection = wallConnectionPerpendicularDirectionIndex(fuelDirectionIndex);
    for (int index = 0; index < static_cast<int>(kSourceDirections.size()); ++index) {
        if (index != centerDirection && index != coolingChainDirection) {
            directions.push_back(index);
        }
    }
    return directions;
}

std::vector<ActivationLine> activationLineOptionsForDirection(
    const Fuel& fuel, int directionIndex, const std::vector<int>& selectedModeratorTypes,
    const std::vector<int>& selectedReflectorTypes) {
    std::vector<ActivationLine> options;
    options.reserve(selectedReflectorTypes.size() *
                    (selectedModeratorTypes.size() +
                     selectedModeratorTypes.size() * (selectedModeratorTypes.size() + 1) / 2));

    for (int firstModerator : selectedModeratorTypes) {
        for (int reflectorType : selectedReflectorTypes) {
            ActivationLine line;
            line.directionIndex = directionIndex;
            line.moderatorTypes = {firstModerator};
            line.reflectorType = reflectorType;
            line.reflectedFlux = reflectedFluxForActivationLine(fuel, line.moderatorTypes, reflectorType);
            options.push_back(std::move(line));
        }
    }

    for (size_t firstOffset = 0; firstOffset < selectedModeratorTypes.size(); ++firstOffset) {
        for (size_t secondOffset = firstOffset; secondOffset < selectedModeratorTypes.size(); ++secondOffset) {
            for (int reflectorType : selectedReflectorTypes) {
                ActivationLine line;
                line.directionIndex = directionIndex;
                line.moderatorTypes = {
                    selectedModeratorTypes.at(firstOffset),
                    selectedModeratorTypes.at(secondOffset),
                };
                line.reflectorType = reflectorType;
                line.reflectedFlux = reflectedFluxForActivationLine(fuel, line.moderatorTypes, reflectorType);
                options.push_back(std::move(line));
            }
        }
    }

    std::sort(options.begin(), options.end(), [](const ActivationLine& lhs, const ActivationLine& rhs) {
        if (lhs.reflectedFlux != rhs.reflectedFlux) {
            return lhs.reflectedFlux < rhs.reflectedFlux;
        }
        if (lhs.moderatorTypes.size() != rhs.moderatorTypes.size()) {
            return lhs.moderatorTypes.size() < rhs.moderatorTypes.size();
        }
        if (lhs.moderatorTypes != rhs.moderatorTypes) {
            return lhs.moderatorTypes < rhs.moderatorTypes;
        }
        return lhs.reflectorType < rhs.reflectorType;
    });
    return options;
}

bool activationLinesWithinReflectorReach(const ActivationPlan& activation) {
    return std::all_of(activation.lines.begin(), activation.lines.end(), [](const ActivationLine& line) {
        return static_cast<int>(line.moderatorTypes.size()) <= kMaxReflectorLineModerators;
    });
}

int activationModeratorCount(const ActivationPlan& plan) {
    int count = 0;
    for (const ActivationLine& line : plan.lines) {
        count += static_cast<int>(line.moderatorTypes.size());
    }
    return count;
}

bool activationLineKeyLess(const ActivationLine& lhs, const ActivationLine& rhs) {
    if (lhs.directionIndex != rhs.directionIndex) {
        return lhs.directionIndex < rhs.directionIndex;
    }
    if (lhs.moderatorTypes.size() != rhs.moderatorTypes.size()) {
        return lhs.moderatorTypes.size() < rhs.moderatorTypes.size();
    }
    if (lhs.moderatorTypes != rhs.moderatorTypes) {
        return lhs.moderatorTypes < rhs.moderatorTypes;
    }
    if (lhs.reflectorType != rhs.reflectorType) {
        return lhs.reflectorType < rhs.reflectorType;
    }
    return lhs.reflectedFlux < rhs.reflectedFlux;
}

bool activationPlanKeyLess(const ActivationPlan& lhs, const ActivationPlan& rhs) {
    return std::lexicographical_compare(lhs.lines.begin(), lhs.lines.end(), rhs.lines.begin(), rhs.lines.end(),
                                        activationLineKeyLess);
}

bool betterActivationPlan(const ActivationPlan& candidate, const ActivationPlan& best) {
    if (candidate.lines.size() != best.lines.size()) {
        return candidate.lines.size() < best.lines.size();
    }
    if (candidate.reflectedFlux != best.reflectedFlux) {
        return candidate.reflectedFlux < best.reflectedFlux;
    }
    const int candidateModeratorCount = activationModeratorCount(candidate);
    const int bestModeratorCount = activationModeratorCount(best);
    if (candidateModeratorCount != bestModeratorCount) {
        return candidateModeratorCount < bestModeratorCount;
    }
    return activationPlanKeyLess(candidate, best);
}

bool partialActivationPlanCannotBeatBest(const ActivationPlan& current, const ActivationPlan& best) {
    if (current.lines.size() > best.lines.size()) {
        return true;
    }
    if (current.lines.size() < best.lines.size()) {
        return false;
    }
    if (current.reflectedFlux > best.reflectedFlux + kActivationFluxEpsilon) {
        return true;
    }
    if (std::abs(current.reflectedFlux - best.reflectedFlux) <= kActivationFluxEpsilon &&
        activationModeratorCount(current) > activationModeratorCount(best)) {
        return true;
    }
    return false;
}

ActivationSearchContext makeActivationSearchContext(const Fuel& fuel, int fuelDirectionIndex,
                                                    const BuildRequest& request,
                                                    const std::atomic_bool* cancelRequested) {
    ActivationSearchContext search;
    search.fuel = &fuel;
    search.cancelRequested = cancelRequested;

    const std::vector<int> directions = activationDirectionsForFuel(fuelDirectionIndex);
    search.optionsByDirection.reserve(directions.size());
    for (int direction : directions) {
        throwIfCancelled(cancelRequested);
        search.optionsByDirection.push_back(
            activationLineOptionsForDirection(fuel, direction, request.selectedModeratorTypeIndices,
                                              request.selectedReflectorTypeIndices));
    }

    search.remainingMaxFlux.assign(search.optionsByDirection.size() + 1, 0.0);
    for (size_t offset = search.optionsByDirection.size(); offset > 0; --offset) {
        const std::vector<ActivationLine>& options = search.optionsByDirection.at(offset - 1);
        const double maxFlux = options.empty() ? 0.0 : std::max(0.0, options.back().reflectedFlux);
        search.remainingMaxFlux.at(offset - 1) = search.remainingMaxFlux.at(offset) + maxFlux;
    }
    return search;
}

void chooseActivationPlanRecursive(const ActivationSearchContext& search, size_t directionOffset,
                                   ActivationPlan& current, std::optional<ActivationPlan>& best) {
    throwIfCancelled(search.cancelRequested);
    const Fuel& fuel = *search.fuel;
    const double maxAllowedFlux = 2.0 * fuel.criticality;
    if (current.reflectedFlux > maxAllowedFlux + kActivationFluxEpsilon) {
        return;
    }
    if (current.reflectedFlux + search.remainingMaxFlux.at(directionOffset) + kActivationFluxEpsilon <
        fuel.criticality) {
        return;
    }
    if (best.has_value() && partialActivationPlanCannotBeatBest(current, *best)) {
        return;
    }
    if (best.has_value() && current.lines.size() == best->lines.size() &&
        current.reflectedFlux + kActivationFluxEpsilon < fuel.criticality) {
        return;
    }

    if (directionOffset >= search.optionsByDirection.size()) {
        if (current.reflectedFlux + kActivationFluxEpsilon < fuel.criticality ||
            current.reflectedFlux > maxAllowedFlux + kActivationFluxEpsilon) {
            return;
        }
        if (!best.has_value() || betterActivationPlan(current, *best)) {
            best = current;
        }
        return;
    }

    chooseActivationPlanRecursive(search, directionOffset + 1, current, best);
    if (best.has_value() && current.lines.size() >= best->lines.size()) {
        return;
    }

    const std::vector<ActivationLine>& options = search.optionsByDirection.at(directionOffset);
    for (const ActivationLine& line : options) {
        throwIfCancelled(search.cancelRequested);
        if (current.reflectedFlux + line.reflectedFlux > maxAllowedFlux + kActivationFluxEpsilon) {
            break;
        }
        current.lines.push_back(line);
        current.reflectedFlux += line.reflectedFlux;
        chooseActivationPlanRecursive(search, directionOffset + 1, current, best);
        current.reflectedFlux -= line.reflectedFlux;
        current.lines.pop_back();
    }
}

std::optional<ActivationPlan> chooseActivationPlanForFuel(int fuelIndex, int fuelDirectionIndex,
                                                          const BuildRequest& request,
                                                          const std::atomic_bool* cancelRequested) {
    const Fuel& fuel = fuels().at(static_cast<size_t>(fuelIndex));
    ActivationSearchContext search =
        makeActivationSearchContext(fuel, fuelDirectionIndex, request, cancelRequested);
    ActivationPlan current;
    std::optional<ActivationPlan> best;
    chooseActivationPlanRecursive(search, 0, current, best);
    return best;
}

bool restoreFixedIrradiatorSkeleton(Grid& grid, const FixedIrradiatorSkeleton& skeleton) {
    for (const FixedBlock& fixed : skeleton.fixedInterior) {
        if (!grid.isInterior(fixed.pos.x, fixed.pos.y, fixed.pos.z)) {
            return false;
        }
        grid.at(fixed.pos.x, fixed.pos.y, fixed.pos.z) = fixed.block;
    }

    for (const Pos& sourcePos : skeleton.sourcePositions) {
        if (!grid.isBoundary(sourcePos.x, sourcePos.y, sourcePos.z)) {
            return false;
        }
        Block& block = grid.at(sourcePos.x, sourcePos.y, sourcePos.z);
        if (block.kind != BlockKind::Casing && block.kind != BlockKind::Source) {
            return false;
        }
        block = {BlockKind::Source, kAnySource};
    }
    for (size_t i = 0; i < skeleton.sourcePositions.size(); ++i) {
        if (i >= skeleton.sourceTargets.size() ||
            !keepSourceLineOpen(grid, skeleton, skeleton.sourcePositions.at(i), skeleton.sourceTargets.at(i))) {
            return false;
        }
    }
    return true;
}

bool activationPlanBlocksSourceDirection(const ActivationPlan& plan, int sourceDirectionIndex) {
    return std::any_of(plan.lines.begin(), plan.lines.end(), [&](const ActivationLine& line) {
        return line.directionIndex == sourceDirectionIndex &&
               reflectorTypes().at(static_cast<size_t>(line.reflectorType)).reflectivity >= 1.0;
    });
}

std::vector<int> sourceDirectionPreference(int fuelDirectionIndex, const ActivationPlan& activation) {
    std::vector<int> directions;
    if (!activationPlanBlocksSourceDirection(activation, fuelDirectionIndex)) {
        directions.push_back(fuelDirectionIndex);
    }
    for (int index = 0; index < static_cast<int>(kSourceDirections.size()); ++index) {
        if (index != fuelDirectionIndex && index != oppositeDirectionIndex(fuelDirectionIndex) &&
            !activationPlanBlocksSourceDirection(activation, index)) {
            directions.push_back(index);
        }
    }
    return directions;
}

std::optional<Pos> findSourceForFuel(const Grid& grid, const FixedIrradiatorSkeleton& skeleton,
                                     int fuelDirectionIndex, const Pos& fuelPos,
                                     const ActivationPlan& activation) {
    for (int sourceDirectionIndex : sourceDirectionPreference(fuelDirectionIndex, activation)) {
        const Direction& sourceDirection = kSourceDirections.at(static_cast<size_t>(sourceDirectionIndex));
        const Pos sourcePos = sourcePositionForDirection(grid, fuelPos, sourceDirection);
        if (!grid.isBoundary(sourcePos.x, sourcePos.y, sourcePos.z)) {
            continue;
        }
        const Block& current = grid.at(sourcePos.x, sourcePos.y, sourcePos.z);
        if (current.kind != BlockKind::Casing && current.kind != BlockKind::Source) {
            continue;
        }

        Grid trial = grid;
        if (!restoreFixedIrradiatorSkeleton(trial, skeleton)) {
            continue;
        }
        trial.at(sourcePos.x, sourcePos.y, sourcePos.z) = {BlockKind::Source, kAnySource};
        const int target = sourcePrimingTargetIndex(trial, sourcePos);
        if (target == trial.index(fuelPos.x, fuelPos.y, fuelPos.z)) {
            return sourcePos;
        }
    }
    return std::nullopt;
}

FixedIrradiatorSkeleton buildBaseSkeleton(const BuildRequest& request, const std::atomic_bool* cancelRequested) {
    if (request.fuelIndices.size() != kIrradiatorFuelInputCount) {
        std::ostringstream os;
        os << "中心辐照仓模式需要 " << kIrradiatorFuelInputCount << " 个燃料单元。";
        throw std::invalid_argument(os.str());
    }

    FixedIrradiatorSkeleton skeleton;
    skeleton.sourceLineFallbackReflectorType = bestWeakSelectedReflectorType(request);
    const int centerModeratorType = strongestSelectedModeratorType(request);
    const Pos center{kIrradiatorCenter, kIrradiatorCenter, kIrradiatorCenter};
    if (!addFixedBlock(skeleton, center, {BlockKind::Irradiator, request.irradiatorRecipeIndex})) {
        throw std::runtime_error("中心辐照仓骨架发生方块冲突。");
    }

    skeleton.fuelPositions.resize(kSourceDirections.size());
    skeleton.activations.resize(kSourceDirections.size());
    size_t fuelInputIndex = 0;
    for (int directionIndex = 0; directionIndex < static_cast<int>(kSourceDirections.size()); ++directionIndex) {
        throwIfCancelled(cancelRequested);
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(directionIndex));
        if (isIrradiatorWallBridgeDirection(directionIndex)) {
            if (!addFixedBlock(skeleton, offset(center, dir, 1), {BlockKind::Conductor, -1})) {
                throw std::runtime_error("中心辐照仓导体连接骨架发生方块冲突。");
            }
            continue;
        }

        for (int distance = 1; distance <= kMaxIrradiatorLineModerators; ++distance) {
            if (!addFixedBlock(skeleton, offset(center, dir, distance),
                               {BlockKind::Moderator, centerModeratorType})) {
                throw std::runtime_error("中心高通量减速剂骨架发生方块冲突。");
            }
        }

        const int fuelIndex = request.fuelIndices.at(fuelInputIndex++);
        const Pos fuelPos = offset(center, dir, kIrradiatorFuelDistance);
        skeleton.fuelPositions.at(static_cast<size_t>(directionIndex)) = fuelPos;
        if (!addFixedBlock(skeleton, fuelPos, {BlockKind::FuelCell, fuelIndex})) {
            throw std::runtime_error("中心辐照仓燃料单元骨架发生方块冲突。");
        }

        std::optional<ActivationPlan> activation =
            chooseActivationPlanForFuel(fuelIndex, directionIndex, request, cancelRequested);
        if (!activation.has_value()) {
            std::ostringstream os;
            const Fuel& fuel = fuels().at(static_cast<size_t>(fuelIndex));
            os << "燃料 " << fuelCodeOrName(fuelIndex)
               << " 无法在中心辐照仓固定骨架中通过动态减速剂 + 反射器组合安全达到临界：临界因子 "
               << fuel.criticality << "，允许上限 " << 2.0 * fuel.criticality
               << "。";
            throw std::runtime_error(os.str());
        }
        if (!activationLinesWithinReflectorReach(*activation)) {
            throw std::runtime_error("燃料外侧动态维持临界反射线超过 2 个减速剂。");
        }
        skeleton.activations.at(static_cast<size_t>(directionIndex)) = *activation;

        for (const ActivationLine& line : activation->lines) {
            const Direction& activationDir = kSourceDirections.at(static_cast<size_t>(line.directionIndex));
            for (size_t offsetIndex = 0; offsetIndex < line.moderatorTypes.size(); ++offsetIndex) {
                if (!addFixedBlock(skeleton, offset(fuelPos, activationDir, static_cast<int>(offsetIndex) + 1),
                                   {BlockKind::Moderator, line.moderatorTypes.at(offsetIndex)})) {
                    throw std::runtime_error("燃料外侧动态维持临界减速剂发生方块冲突。");
                }
            }
            const Pos reflectorPos = offset(fuelPos, activationDir,
                                            static_cast<int>(line.moderatorTypes.size()) + 1);
            if (!addFixedBlock(skeleton, reflectorPos, {BlockKind::Reflector, line.reflectorType})) {
                throw std::runtime_error("燃料外侧动态维持临界反射器发生方块冲突。");
            }
        }
        addWallConnectionChain(skeleton, fuelPos, dir, directionIndex);
    }

    return skeleton;
}

Grid buildIrradiatorSkeletonGrid(const BuildRequest& request, FixedIrradiatorSkeleton& skeleton) {
    Grid grid = makeShell(kIrradiatorInteriorSize, kIrradiatorInteriorSize, kIrradiatorInteriorSize);
    if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("无法恢复中心辐照仓固定骨架。");
    }

    size_t fuelInputIndex = 0;
    for (int directionIndex = 0; directionIndex < static_cast<int>(kSourceDirections.size()); ++directionIndex) {
        if (isIrradiatorWallBridgeDirection(directionIndex)) {
            continue;
        }

        const int fuelIndex = request.fuelIndices.at(fuelInputIndex++);
        const Fuel& fuel = fuels().at(static_cast<size_t>(fuelIndex));
        if (fuel.selfPriming) {
            continue;
        }

        const Pos fuelPos = skeleton.fuelPositions.at(static_cast<size_t>(directionIndex));
        const ActivationPlan& activation = skeleton.activations.at(static_cast<size_t>(directionIndex));
        std::optional<Pos> sourcePos = findSourceForFuel(grid, skeleton, directionIndex, fuelPos, activation);
        if (!sourcePos.has_value()) {
            std::ostringstream os;
            os << "无法为非自启动燃料 " << fuelCodeOrName(fuelIndex)
               << " 放置能够指向该燃料的外壁中子源。";
            throw std::runtime_error(os.str());
        }
        grid.at(sourcePos->x, sourcePos->y, sourcePos->z) = {BlockKind::Source, kAnySource};
        skeleton.sourcePositions.push_back(*sourcePos);
        skeleton.sourceTargets.push_back(fuelPos);
    }

    if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("无法恢复带中子源的中心辐照仓固定骨架。");
    }
    return grid;
}

void requireSixFuelIrradiatorState(const Grid& grid, const FuelSimulation& sim, const char* stage) {
    if (!isPreCompactRunnable(sim)) {
        std::ostringstream os;
        os << "中心辐照仓方案在 " << stage << " 阶段无法让 " << kIrradiatorFuelInputCount
           << " 个燃料全部运行（运行 "
           << sim.runningCells << "/" << sim.fuelCells << "）。";
        throw std::runtime_error(os.str());
    }
    if (!hasSafeFuelFlux(grid, sim)) {
        std::ostringstream os;
        os << "中心辐照仓方案在 " << stage << " 阶段超过燃料安全通量限制。";
        throw std::runtime_error(os.str());
    }
    if (countFunctionalIrradiators(sim) != 1) {
        std::ostringstream os;
        os << "中心辐照仓方案在 " << stage << " 阶段未形成唯一有效辐照仓。";
        throw std::runtime_error(os.str());
    }
}

bool isAcceptedSixFuelIrradiator(const Grid& grid, const FuelSimulation& sim) {
    return isSafeOperatingSimulation(grid, sim) && countFunctionalIrradiators(sim) == 1;
}

void clearMutableSupport(Grid& grid, const FixedIrradiatorSkeleton& skeleton) {
    for (const Pos& pos : grid.interiorPositions()) {
        if (!isFixedInteriorPosition(skeleton, pos) && isSupportMutable(grid.at(pos.x, pos.y, pos.z))) {
            grid.at(pos.x, pos.y, pos.z) = {BlockKind::Empty, -1};
        }
    }
}

Grid expandIrradiatorCooling(Grid grid, const FixedIrradiatorSkeleton& skeleton,
                             const std::atomic_bool* cancelRequested) {
    return expandCoolingWithPreserver(
        std::move(grid),
        [&](Grid& candidate) {
            return restoreFixedIrradiatorSkeleton(candidate, skeleton);
        },
        cancelRequested, kIrradiatorCoolingExpansionOptions);
}

} // namespace

OptimizationResult optimizeSixFuelIrradiatorLayout(const BuildRequest& request,
                                                   const std::atomic_bool* cancelRequested) {
    NCFR_PERF_COUNT(finalizeCandidateCalls);
    throwIfCancelled(cancelRequested);
    const SupportBlockOptions supportOptions{
        request.selectedModeratorTypeIndices,
        request.selectedReflectorTypeIndices,
    };
    FixedIrradiatorSkeleton skeleton = buildBaseSkeleton(request, cancelRequested);
    Grid grid = buildIrradiatorSkeletonGrid(request, skeleton);

    FuelSimulation sim = simulateMixedFuel(grid);
    requireSixFuelIrradiatorState(grid, sim, "初始骨架");

    fillSupportBlocks(grid, &supportOptions);
    if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("放置基础支撑块后无法恢复中心辐照仓固定骨架。");
    }
    pruneInactiveSupport(grid);
    if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("清理无效支撑块后无法恢复中心辐照仓固定骨架。");
    }
    sim = simulateMixedFuel(grid);
    requireSixFuelIrradiatorState(grid, sim, "基础支撑剪枝后");
    if (isAcceptedSixFuelIrradiator(grid, sim)) {
        return resultFromSimulation(std::move(grid), request, sim);
    }

    if (!sim.compatible || sim.minClusterMargin < 0 || sim.disconnectedFunctionalBlocks != 0) {
        if (!sim.compatible || sim.disconnectedFunctionalBlocks != 0) {
            clearMutableSupport(grid, skeleton);
            if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
                throw std::runtime_error("准备散热器连接路径后无法恢复中心辐照仓固定骨架。");
            }
        }
        grid = expandIrradiatorCooling(std::move(grid), skeleton, cancelRequested);
        if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
            throw std::runtime_error("扩展散热器后无法恢复中心辐照仓固定骨架。");
        }
        pruneInactiveSupport(grid);
        if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
            throw std::runtime_error("扩展散热器清理后无法恢复中心辐照仓固定骨架。");
        }
        sim = simulateMixedFuel(grid);
    }

    if (!isAcceptedSixFuelIrradiator(grid, sim)) {
        grid = improveSupportBlocks(std::move(grid), cancelRequested, kIrradiatorImproveOptions,
                                    &supportOptions);
        if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
            throw std::runtime_error("小范围优化支撑块后无法恢复中心辐照仓固定骨架。");
        }
        pruneInactiveSupport(grid);
        if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
            throw std::runtime_error("小范围优化清理后无法恢复中心辐照仓固定骨架。");
        }
        sim = simulateMixedFuel(grid);
        requireSixFuelIrradiatorState(grid, sim, "小范围支撑优化后");

        if (!isAcceptedSixFuelIrradiator(grid, sim)) {
            grid = expandIrradiatorCooling(std::move(grid), skeleton, cancelRequested);
            if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
                throw std::runtime_error("补救扩展散热器后无法恢复中心辐照仓固定骨架。");
            }
            pruneInactiveSupport(grid);
            if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
                throw std::runtime_error("补救扩展清理后无法恢复中心辐照仓固定骨架。");
            }
            sim = simulateMixedFuel(grid);
        }
    }

    requireSixFuelIrradiatorState(grid, sim, "最终验证");
    if (!isAcceptedSixFuelIrradiator(grid, sim)) {
        std::ostringstream os;
        os << "无法在中心辐照仓固定骨架周围放置足够散热器；最小散热余量 "
           << sim.minClusterMargin << " H/t，断开功能块 " << sim.disconnectedFunctionalBlocks << "。";
        throw std::runtime_error(os.str());
    }

    return resultFromSimulation(std::move(grid), request, sim);
}

} // namespace ncfr::optimizer_detail
