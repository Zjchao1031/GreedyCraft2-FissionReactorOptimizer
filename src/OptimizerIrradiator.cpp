#include "OptimizerIrradiator.h"

#include "OptimizerCommon.h"
#include "OptimizerConductorBridge.h"
#include "OptimizerCooling.h"
#include "OptimizerDiagnostics.h"
#include "OptimizerDirectional.h"
#include "OptimizerSpecialCooling.h"
#include "FuelSpecialCases.h"
#include "NeutronRules.h"
#include "Perf.h"

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

constexpr int kIrradiatorInteriorSize = kMaxSize;
constexpr int kIrradiatorCenter = 12;
constexpr int kIrradiatorFuelInputCount = 5;
constexpr int kIrradiatorFuelDistance = kMaxIrradiatorLineModerators + 1;
constexpr long long kIrradiatorManaDustCoolingHandoffThreshold = 12265;
constexpr int kIrradiatorAboveDirectionIndex = 2;
constexpr int kIrradiatorReservedDirectionIndex = 3;
constexpr int kIrradiatorRequiredActivationDirectionIndex = kIrradiatorAboveDirectionIndex;
constexpr int kAnySource = -1;
constexpr double kActivationFluxEpsilon = 1e-9;
constexpr ImproveOptions kIrradiatorImproveOptions{1, 1, 64};
constexpr CoolingExpansionOptions kIrradiatorCoolingExpansionOptions{12, 2, 192, 3, 192, 64, 64, 3};
constexpr Direction kIrradiatorReservedDirection{0, -1, 0};

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
    std::vector<int> fuelRequestSlots;
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

std::string fuelName(int fuelIndex) {
    const Fuel& fuel = fuels().at(static_cast<size_t>(fuelIndex));
    return fuel.nameZh;
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

bool isIrradiatorReservedDirection(int directionIndex) {
    return sameDirection(kSourceDirections.at(static_cast<size_t>(directionIndex)),
                         kIrradiatorReservedDirection);
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
    std::vector<int> directions{kIrradiatorRequiredActivationDirectionIndex};
    const int centerDirection = oppositeDirectionIndex(fuelDirectionIndex);
    const int coolingChainDirection = wallConnectionPerpendicularDirectionIndex(fuelDirectionIndex);
    for (int index = 0; index < static_cast<int>(kSourceDirections.size()); ++index) {
        if (index != kIrradiatorRequiredActivationDirectionIndex &&
            index != centerDirection && index != coolingChainDirection) {
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

bool betterActivationPlan(const ActivationPlan& candidate, const ActivationPlan& best) {
    if (candidate.lines.size() != best.lines.size()) {
        return candidate.lines.size() < best.lines.size();
    }
    const int candidateModeratorCount = activationModeratorCount(candidate);
    const int bestModeratorCount = activationModeratorCount(best);
    return candidateModeratorCount > bestModeratorCount;
}

bool partialActivationPlanCannotBeatBest(const ActivationPlan& current, const ActivationPlan& best) {
    if (current.lines.size() > best.lines.size()) {
        return true;
    }
    if (current.lines.size() < best.lines.size()) {
        return false;
    }
    return activationModeratorCount(current) <= activationModeratorCount(best);
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
    std::optional<ActivationPlan> best;
    const std::vector<ActivationLine>& requiredOptions = search.optionsByDirection.front();
    for (const ActivationLine& requiredLine : requiredOptions) {
        throwIfCancelled(cancelRequested);
        ActivationPlan current;
        current.lines.push_back(requiredLine);
        current.reflectedFlux = requiredLine.reflectedFlux;
        chooseActivationPlanRecursive(search, 1, current, best);
    }
    return best;
}

std::vector<int> irradiatorFuelRequestSlots(const BuildRequest& request) {
    int highestHeatSlot = 0;
    for (int slot = 1; slot < static_cast<int>(request.fuelIndices.size()); ++slot) {
        const Fuel& candidate = fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(slot))));
        const Fuel& currentHighest =
            fuels().at(static_cast<size_t>(request.fuelIndices.at(static_cast<size_t>(highestHeatSlot))));
        if (candidate.heat > currentHighest.heat) {
            highestHeatSlot = slot;
        }
    }

    std::vector<int> requestSlots(kSourceDirections.size(), -1);
    requestSlots.at(kIrradiatorAboveDirectionIndex) = highestHeatSlot;
    int remainingDirectionIndex = 0;
    for (int slot = 0; slot < static_cast<int>(request.fuelIndices.size()); ++slot) {
        if (slot == highestHeatSlot) {
            continue;
        }
        while (remainingDirectionIndex == kIrradiatorAboveDirectionIndex ||
               remainingDirectionIndex == kIrradiatorReservedDirectionIndex) {
            ++remainingDirectionIndex;
        }
        requestSlots.at(static_cast<size_t>(remainingDirectionIndex++)) = slot;
    }
    return requestSlots;
}

int irradiatorFuelRequestSlot(const FixedIrradiatorSkeleton& skeleton, int directionIndex) {
    return skeleton.fuelRequestSlots.at(static_cast<size_t>(directionIndex));
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
    skeleton.fuelRequestSlots.assign(kSourceDirections.size(), -1);
    skeleton.activations.resize(kSourceDirections.size());
    const std::vector<int> fuelRequestSlots = irradiatorFuelRequestSlots(request);
    for (int directionIndex = 0; directionIndex < static_cast<int>(kSourceDirections.size()); ++directionIndex) {
        throwIfCancelled(cancelRequested);
        const Direction& dir = kSourceDirections.at(static_cast<size_t>(directionIndex));
        if (isIrradiatorReservedDirection(directionIndex)) {
            continue;
        }

        for (int distance = 1; distance <= kMaxIrradiatorLineModerators; ++distance) {
            if (!addFixedBlock(skeleton, offset(center, dir, distance),
                               {BlockKind::Moderator, centerModeratorType})) {
                throw std::runtime_error("中心高通量减速剂骨架发生方块冲突。");
            }
        }

        const int requestSlot = fuelRequestSlots.at(static_cast<size_t>(directionIndex));
        const int fuelIndex = request.fuelIndices.at(static_cast<size_t>(requestSlot));
        skeleton.fuelRequestSlots.at(static_cast<size_t>(directionIndex)) = requestSlot;
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
#ifndef NDEBUG
            {
                std::ostringstream detail;
                detail << "reason=requiredPlusYActivationLine"
                       << " requestSlot=" << requestSlot
                       << " fuel=" << fuel.nameZh
                       << " fuelDirection=" << directionIndex
                       << " requiredDirection="
                       << kIrradiatorRequiredActivationDirectionIndex
                       << " criticality=" << fuel.criticality
                       << " maxFlux=" << 2.0 * fuel.criticality;
                NCFR_PERF_CHECKPOINT("irradiatorActivation", detail.str().c_str());
            }
#endif
            os << "燃料 " << fuelName(fuelIndex)
               << " 无法在中心辐照仓固定骨架中通过必选 +Y 首条减速剂线与反射器组合安全达到临界：临界因子 "
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
    }

    return skeleton;
}

Grid buildIrradiatorSkeletonGrid(const BuildRequest& request, FixedIrradiatorSkeleton& skeleton) {
    Grid grid = makeShell(kIrradiatorInteriorSize, kIrradiatorInteriorSize, kIrradiatorInteriorSize);
    if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("无法恢复中心辐照仓固定骨架。");
    }

    for (int directionIndex = 0; directionIndex < static_cast<int>(kSourceDirections.size()); ++directionIndex) {
        if (isIrradiatorReservedDirection(directionIndex)) {
            continue;
        }

        const int requestSlot = irradiatorFuelRequestSlot(skeleton, directionIndex);
        const int fuelIndex = request.fuelIndices.at(static_cast<size_t>(requestSlot));
        const Fuel& fuel = fuels().at(static_cast<size_t>(fuelIndex));
        if (fuel.selfPriming) {
            continue;
        }

        const Pos fuelPos = skeleton.fuelPositions.at(static_cast<size_t>(directionIndex));
        const ActivationPlan& activation = skeleton.activations.at(static_cast<size_t>(directionIndex));
        std::optional<Pos> sourcePos = findSourceForFuel(grid, skeleton, directionIndex, fuelPos, activation);
        if (!sourcePos.has_value()) {
            std::ostringstream os;
            os << "无法为非自启动燃料 " << fuelName(fuelIndex)
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

void requireFiveFuelIrradiatorState(const Grid& grid, const FuelSimulation& sim, const char* stage) {
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

bool isAcceptedFiveFuelIrradiator(const Grid& grid, const FuelSimulation& sim) {
    return isOverallCoolingOperatingSimulation(grid, sim) &&
           countFunctionalIrradiators(sim) == 1;
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
        cancelRequested, kIrradiatorCoolingExpansionOptions, false,
        CoolingValidationPolicy::Overall);
}

FuelLineSpec specialCoolingFuelLine(const ActivationLine& line) {
    return {
        line.directionIndex,
        static_cast<int>(line.moderatorTypes.size()),
        line.moderatorTypes.front(),
        line.reflectorType,
        line.reflectedFlux,
    };
}

bool isBetterSpecialCoolingAnchor(const FuelLineSpec& candidate,
                                  size_t candidateEmptyPositions,
                                  const FuelLineSpec& current,
                                  size_t currentEmptyPositions) {
    if (candidateEmptyPositions != currentEmptyPositions) {
        return candidateEmptyPositions > currentEmptyPositions;
    }
    if (candidate.moderatorCount != current.moderatorCount) {
        return candidate.moderatorCount < current.moderatorCount;
    }
    if (candidate.direction != current.direction) {
        return candidate.direction < current.direction;
    }
    return candidate.reflectorType < current.reflectorType;
}

std::vector<FuelLayoutContext> irradiatorSpecialCoolingContexts(
    const Grid& grid, const FixedIrradiatorSkeleton& skeleton) {
    std::vector<FuelLayoutContext> contexts;
    contexts.reserve(kIrradiatorFuelInputCount);

    for (int directionIndex = 0;
         directionIndex < static_cast<int>(kSourceDirections.size());
         ++directionIndex) {
        if (isIrradiatorReservedDirection(directionIndex)) {
            continue;
        }

        const Pos fuelPos =
            skeleton.fuelPositions.at(static_cast<size_t>(directionIndex));
        const ActivationPlan& activation =
            skeleton.activations.at(static_cast<size_t>(directionIndex));
        std::optional<FuelLineSpec> bestLine;
        size_t bestEmptyPositions = 0;
        for (const ActivationLine& line : activation.lines) {
            if (line.moderatorTypes.empty()) {
                continue;
            }
            const FuelLineSpec candidate = specialCoolingFuelLine(line);
            const auto endStoneCandidates =
                endStoneReflectorSinkCandidates(
                    grid, fuelPos, {candidate});
            if (!endStoneCandidates.has_value()) {
                continue;
            }
            const size_t emptyPositions = static_cast<size_t>(
                std::count_if(
                    endStoneCandidates->begin(),
                    endStoneCandidates->end(),
                    [&](const EndStoneReflectorCandidate& value) {
                        return grid.at(
                                   value.pos.x, value.pos.y,
                                   value.pos.z).kind ==
                               BlockKind::Empty;
                    }));
            if (!bestLine.has_value() ||
                isBetterSpecialCoolingAnchor(
                    candidate, emptyPositions, *bestLine,
                    bestEmptyPositions)) {
                bestLine = candidate;
                bestEmptyPositions = emptyPositions;
            }
        }

        FuelLayoutContext context;
        context.requestSlot =
            irradiatorFuelRequestSlot(skeleton, directionIndex);
        context.fuelPos = fuelPos;
        if (bestLine.has_value()) {
            context.fuelLines.push_back(*bestLine);
        }
        contexts.push_back(std::move(context));
    }
    return contexts;
}

#ifndef NDEBUG
void logIrradiatorSpecialCoolingCheckpoint(
    const char* stage, const char* outcome, const Grid& grid,
    const FuelSimulation& sim, long long deficit) {
    std::ostringstream os;
    os << "stage=" << stage
       << " outcome=" << outcome
       << " grid=" << gridInteriorLabel(grid)
       << " rawHeating=" << sim.rawHeating
       << " cooling=" << sim.cooling
       << " deficit=" << deficit
       << " minMargin=" << sim.minClusterMargin
       << " disconnected=" << sim.disconnectedFunctionalBlocks;
    NCFR_PERF_CHECKPOINT("irradiatorSpecialCooling", os.str().c_str());
}
#endif

std::optional<Grid> tryIrradiatorSpecialCoolingFallback(
    const Grid& grid, const FuelSimulation& sim,
    const FixedIrradiatorSkeleton& skeleton,
    const BuildRequest& request, const char* stage,
    const std::atomic_bool* cancelRequested) {
    const long long deficit = sim.rawHeating - sim.cooling;
    const MixedFuelSpecialCoolingLimits limits =
        mixedFuelSpecialCoolingLimits(kIrradiatorFuelInputCount);
    if (!isPreCompactRunnable(sim) ||
        !hasSafeFuelFlux(grid, sim) ||
        hasInvalidSinks(grid, sim) ||
        deficit <= 0 ||
        deficit > limits.manaDustDeficitLimit) {
#ifndef NDEBUG
        logIrradiatorSpecialCoolingCheckpoint(
            stage, "skipped", grid, sim, deficit);
#endif
        return std::nullopt;
    }

    const std::vector<FuelLayoutContext> contexts =
        irradiatorSpecialCoolingContexts(grid, skeleton);
#ifndef NDEBUG
    logIrradiatorSpecialCoolingCheckpoint(
        stage, "attempt", grid, sim, deficit);
#endif
    std::optional<Grid> special =
        tryMixedFuelSpecialCoolingFallback(
            grid, request, contexts, cancelRequested, true,
            CoolingValidationPolicy::Overall,
            kIrradiatorManaDustCoolingHandoffThreshold);
    if (!special.has_value()) {
#ifndef NDEBUG
        logIrradiatorSpecialCoolingCheckpoint(
            stage, "rejected", grid, sim, deficit);
#endif
        return std::nullopt;
    }

    const FuelSimulation specialSim = simulateMixedFuel(*special);
    if (!isPreCompactRunnable(specialSim) ||
        !isAcceptedFiveFuelIrradiator(*special, specialSim) ||
        hasInvalidSinks(*special, specialSim)) {
#ifndef NDEBUG
        logIrradiatorSpecialCoolingCheckpoint(
            stage, "invalidResult", *special, specialSim,
            specialSim.rawHeating - specialSim.cooling);
#endif
        return std::nullopt;
    }
#ifndef NDEBUG
    logIrradiatorSpecialCoolingCheckpoint(
        stage, "accepted", *special, specialSim,
        specialSim.rawHeating - specialSim.cooling);
#endif
    return special;
}

} // namespace

OptimizationResult optimizeFiveFuelIrradiatorLayout(const BuildRequest& request,
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
    requireFiveFuelIrradiatorState(grid, sim, "初始骨架");

    fillSupportBlocks(grid, &supportOptions);
    if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("放置基础支撑块后无法恢复中心辐照仓固定骨架。");
    }
    pruneInactiveSupport(grid);
    if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("清理无效支撑块后无法恢复中心辐照仓固定骨架。");
    }
    sim = simulateMixedFuel(grid);
    requireFiveFuelIrradiatorState(grid, sim, "基础支撑剪枝后");
    if (isAcceptedFiveFuelIrradiator(grid, sim) &&
        evaluateHeatingClusterWallConnections(grid, sim).allConnected()) {
        return resultFromSimulation(std::move(grid), request, sim);
    }

    bool specialCoolingApplied = false;
    if (std::optional<Grid> special =
            tryIrradiatorSpecialCoolingFallback(
                grid, sim, skeleton, request, "baseSupport",
                cancelRequested)) {
        grid = std::move(*special);
        sim = simulateMixedFuel(grid);
        specialCoolingApplied = true;
    }

    if (!specialCoolingApplied && !isAcceptedFiveFuelIrradiator(grid, sim)) {
        if (!isPreCompactRunnable(sim) || sim.disconnectedFunctionalBlocks != 0) {
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

    if (!specialCoolingApplied) {
        if (std::optional<Grid> special =
                tryIrradiatorSpecialCoolingFallback(
                    grid, sim, skeleton, request, "afterCoolingExpansion",
                    cancelRequested)) {
            grid = std::move(*special);
            sim = simulateMixedFuel(grid);
            specialCoolingApplied = true;
        }
    }

    if (!specialCoolingApplied &&
        !isAcceptedFiveFuelIrradiator(grid, sim)) {
        grid = improveSupportBlocks(std::move(grid), cancelRequested, kIrradiatorImproveOptions,
                                    &supportOptions, nullptr, false,
                                    CoolingValidationPolicy::Overall);
        if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
            throw std::runtime_error("小范围优化支撑块后无法恢复中心辐照仓固定骨架。");
        }
        pruneInactiveSupport(grid);
        if (!restoreFixedIrradiatorSkeleton(grid, skeleton)) {
            throw std::runtime_error("小范围优化清理后无法恢复中心辐照仓固定骨架。");
        }
        sim = simulateMixedFuel(grid);
        requireFiveFuelIrradiatorState(grid, sim, "小范围支撑优化后");

        if (!isAcceptedFiveFuelIrradiator(grid, sim)) {
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

    if (!specialCoolingApplied) {
        if (std::optional<Grid> special =
                tryIrradiatorSpecialCoolingFallback(
                    grid, sim, skeleton, request, "afterSupportRecovery",
                    cancelRequested)) {
            grid = std::move(*special);
            sim = simulateMixedFuel(grid);
            specialCoolingApplied = true;
        }
    }

    requireFiveFuelIrradiatorState(grid, sim, "导体桥接前");
    grid = compactEmptyInteriorPlanes(std::move(grid));
    sim = simulateMixedFuel(grid);
    const StateVector protectedPositions = protectFuelLineBlocks(grid);
    constexpr ConductorBridgeCoolingPolicy coolingPolicy =
        ConductorBridgeCoolingPolicy::Overall;
    if (canAttemptConductorBridge(grid, sim, coolingPolicy)) {
        ConductorBridgeResult bridge =
            connectHeatingClustersWithConductors(
                grid, sim, &protectedPositions, cancelRequested,
                coolingPolicy);
#ifndef NDEBUG
        logConductorBridgeCheckpoint(
            bridge.reason.c_str(), bridge.grid, bridge.sim,
            bridge.clusterCount, bridge.conductorsAdded,
            coolingPolicy);
#endif
        if (!bridge.success) {
            std::ostringstream os;
            os << "中心辐照仓导体桥接失败：" << bridge.reason << "。";
            throw std::runtime_error(os.str());
        }
        grid = std::move(bridge.grid);
        sim = std::move(bridge.sim);
    }

    requireFiveFuelIrradiatorState(grid, sim, "最终验证");
    if (!isAcceptedFiveFuelIrradiator(grid, sim)) {
        std::ostringstream os;
        os << "无法在中心辐照仓周围放置足够散热器或连接功能热簇；总体散热余量 "
           << overallCoolingMargin(sim) << " H/t，断开功能块 "
           << sim.disconnectedFunctionalBlocks << "。";
        throw std::runtime_error(os.str());
    }

#ifndef NDEBUG
    {
        std::ostringstream os;
        os << "mode=irradiator grid=" << gridInteriorLabel(grid)
           << " compatible=" << (sim.compatible ? 1 : 0)
           << " rawHeating=" << sim.rawHeating
           << " cooling=" << sim.cooling
           << " coolingMargin=" << overallCoolingMargin(sim)
           << " minMargin=" << sim.minClusterMargin
           << " disconnected=" << sim.disconnectedFunctionalBlocks;
        NCFR_PERF_CHECKPOINT("simulation.search", os.str().c_str());
    }
#endif
    return resultFromSimulation(std::move(grid), request, sim);
}

} // namespace ncfr::optimizer_detail

namespace ncfr {

long long irradiatorInputRawHeating(const BuildRequest& request) {
    const optimizer_detail::SupportBlockOptions supportOptions{
        request.selectedModeratorTypeIndices,
        request.selectedReflectorTypeIndices,
    };
    optimizer_detail::FixedIrradiatorSkeleton skeleton =
        optimizer_detail::buildBaseSkeleton(request, nullptr);
    Grid grid = optimizer_detail::buildIrradiatorSkeletonGrid(request, skeleton);
    FuelSimulation sim = simulateMixedFuel(grid);
    optimizer_detail::requireFiveFuelIrradiatorState(grid, sim, "输入预检");

    optimizer_detail::fillSupportBlocks(grid, &supportOptions);
    if (!optimizer_detail::restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("输入预检后无法恢复中心辐照仓固定骨架。");
    }
    optimizer_detail::pruneInactiveSupport(grid);
    if (!optimizer_detail::restoreFixedIrradiatorSkeleton(grid, skeleton)) {
        throw std::runtime_error("输入预检清理后无法恢复中心辐照仓固定骨架。");
    }
    sim = simulateMixedFuel(grid);
    optimizer_detail::requireFiveFuelIrradiatorState(grid, sim, "输入预检支撑块剪枝后");
    return sim.rawHeating;
}

} // namespace ncfr
