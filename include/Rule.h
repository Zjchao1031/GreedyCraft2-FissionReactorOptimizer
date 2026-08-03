#pragma once

#include "Grid.h"
#include "StateVector.h"

#include <string>
#include <vector>

namespace ncfr {

enum class CountType {
    AtLeast,
    Exactly,
    AtMost
};

enum class RuleOp {
    Leaf,
    And,
    Or
};

enum class RuleTarget {
    Unknown,
    Cell,
    Moderator,
    Reflector,
    Shield,
    Irradiator,
    Conductor,
    Casing,
    AnySink,
    Sink
};

struct Requirement {
    int count = 1;
    CountType countType = CountType::AtLeast;
    bool axial = false;
    RuleTarget target = RuleTarget::Unknown;
    int sinkType = -1;
};

struct RuleNode {
    RuleOp op = RuleOp::Leaf;
    Requirement requirement;
    std::vector<RuleNode> children;
};

struct PlacementRule {
    RuleNode root;
};

struct RuleContext {
    const StateVector* validSinks = nullptr;
    const StateVector* functionalCells = nullptr;
    const StateVector* activeModerators = nullptr;
    const StateVector* activeReflectors = nullptr;
    const StateVector* functionalShields = nullptr;
    const StateVector* functionalIrradiators = nullptr;
};

PlacementRule parsePlacementRule(const std::string& text);
bool isSinkValidAt(const Grid& grid, const Pos& pos, const RuleContext& context);
StateVector evaluateValidSinks(const Grid& grid, const RuleContext& context);

} // namespace ncfr
