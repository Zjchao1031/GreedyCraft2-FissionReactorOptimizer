#include "Rule.h"

#include "Perf.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>

namespace ncfr {
namespace {

std::string trim(std::string s) {
    auto notSpace = [](unsigned char ch) { return !std::isspace(ch); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

std::string normalized(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return trim(s);
}

std::vector<std::string> split(const std::string& text, const std::string& delimiter) {
    std::vector<std::string> parts;
    size_t start = 0;
    while (start < text.size()) {
        const size_t pos = text.find(delimiter, start);
        if (pos == std::string::npos) {
            parts.push_back(trim(text.substr(start)));
            break;
        }
        parts.push_back(trim(text.substr(start, pos - start)));
        start = pos + delimiter.size();
    }
    return parts;
}

std::vector<std::string> words(std::string text) {
    std::vector<std::string> result;
    size_t start = 0;
    while (start < text.size()) {
        while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start]))) {
            ++start;
        }
        if (start >= text.size()) {
            break;
        }
        size_t end = start;
        while (end < text.size() && !std::isspace(static_cast<unsigned char>(text[end]))) {
            ++end;
        }
        result.push_back(text.substr(start, end - start));
        start = end;
    }
    return result;
}

int numberValue(const std::string& token) {
    static const std::unordered_map<std::string, int> numbers = {
        {"zero", 0}, {"one", 1}, {"two", 2}, {"three", 3}, {"four", 4}, {"five", 5},
        {"six", 6}, {"seven", 7}, {"eight", 8}, {"nine", 9}, {"ten", 10},
    };
    if (auto it = numbers.find(token); it != numbers.end()) {
        return it->second;
    }
    if (!token.empty() && std::all_of(token.begin(), token.end(), [](unsigned char ch) { return std::isdigit(ch); })) {
        return std::stoi(token);
    }
    return -1;
}

std::string sinkTypeBefore(const std::vector<std::string>& tokens, size_t sinkIndex) {
    if (sinkIndex == 0) {
        return "any";
    }
    std::string token = tokens.at(sinkIndex - 1);
    if (token == "any") {
        return token;
    }
    return token;
}

int sinkTypeIndexForSourceName(const std::string& sourceName) {
    static const std::unordered_map<std::string, int> indices = [] {
        std::unordered_map<std::string, int> map;
        for (const SinkType& sink : sinkTypes()) {
            map.emplace(sink.sourceName, sink.index);
        }
        return map;
    }();
    if (auto it = indices.find(sourceName); it != indices.end()) {
        return it->second;
    }
    return -1;
}

void setTarget(Requirement& requirement, const std::string& target) {
    if (target == "cell") {
        requirement.target = RuleTarget::Cell;
    } else if (target == "moderator") {
        requirement.target = RuleTarget::Moderator;
    } else if (target == "reflector") {
        requirement.target = RuleTarget::Reflector;
    } else if (target == "shield") {
        requirement.target = RuleTarget::Shield;
    } else if (target == "irradiator") {
        requirement.target = RuleTarget::Irradiator;
    } else if (target == "conductor") {
        requirement.target = RuleTarget::Conductor;
    } else if (target == "casing") {
        requirement.target = RuleTarget::Casing;
    } else if (target == "any") {
        requirement.target = RuleTarget::AnySink;
    } else {
        const int sinkType = sinkTypeIndexForSourceName(target);
        if (sinkType >= 0) {
            requirement.target = RuleTarget::Sink;
            requirement.sinkType = sinkType;
        }
    }
}

Requirement parseRequirement(std::string text) {
    text = normalized(std::move(text));

    Requirement requirement;
    requirement.countType = CountType::AtLeast;
    if (text.find("at most") != std::string::npos) {
        requirement.countType = CountType::AtMost;
    } else if (text.find("exactly") != std::string::npos || text.find("exact") != std::string::npos) {
        requirement.countType = CountType::Exactly;
    }
    requirement.axial = text.find("axial") != std::string::npos || text.find("axially") != std::string::npos;

    const std::vector<std::string> tokens = words(text);
    for (const std::string& token : tokens) {
        const int number = numberValue(token);
        if (number >= 0) {
            requirement.count = number;
            break;
        }
    }

    for (size_t i = 0; i < tokens.size(); ++i) {
        const std::string& token = tokens.at(i);
        if (token.find("wall") != std::string::npos || token.find("casing") != std::string::npos) {
            setTarget(requirement, "casing");
            break;
        }
        if (token.find("conductor") != std::string::npos) {
            setTarget(requirement, "conductor");
            break;
        }
        if (token.find("moderator") != std::string::npos) {
            setTarget(requirement, "moderator");
            break;
        }
        if (token.find("reflector") != std::string::npos) {
            setTarget(requirement, "reflector");
            break;
        }
        if (token.find("irradiator") != std::string::npos) {
            setTarget(requirement, "irradiator");
            break;
        }
        if (token.find("shield") != std::string::npos) {
            setTarget(requirement, "shield");
            break;
        }
        if (token.find("cell") != std::string::npos) {
            setTarget(requirement, "cell");
            break;
        }
        if (token.find("sink") != std::string::npos) {
            setTarget(requirement, sinkTypeBefore(tokens, i));
            break;
        }
    }

    if (requirement.target == RuleTarget::Unknown) {
        throw std::invalid_argument("无法解析散热器摆放规则: " + text);
    }
    return requirement;
}

RuleNode parseNode(const std::string& text) {
    const std::string rule = normalized(text);
    const bool hasAnd = rule.find("&&") != std::string::npos;
    const bool hasOr = rule.find("||") != std::string::npos;
    if (hasAnd && hasOr) {
        throw std::invalid_argument("散热器摆放规则不能同时混用 && 与 ||: " + text);
    }

    if (hasAnd || hasOr) {
        RuleNode node;
        node.op = hasAnd ? RuleOp::And : RuleOp::Or;
        for (const std::string& part : split(rule, hasAnd ? "&&" : "||")) {
            if (!part.empty()) {
                node.children.push_back(parseNode(part));
            }
        }
        return node;
    }

    RuleNode node;
    node.op = RuleOp::Leaf;
    node.requirement = parseRequirement(rule);
    return node;
}

struct RuleTopology {
    std::vector<std::array<int, 6>> neighbors;
    std::vector<std::array<int, 6>> axialPairs;
};

RuleTopology buildRuleTopology(const Grid& grid) {
    RuleTopology topology;
    topology.neighbors.resize(static_cast<size_t>(grid.volume()));
    topology.axialPairs.resize(static_cast<size_t>(grid.volume()));
    for (int idx = 0; idx < grid.volume(); ++idx) {
        topology.neighbors.at(static_cast<size_t>(idx)).fill(-1);
        topology.axialPairs.at(static_cast<size_t>(idx)).fill(-1);
    }

    static constexpr int neighborDirs[6][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    static constexpr int axialDirs[3][3] = {
        {1, 0, 0}, {0, 1, 0}, {0, 0, 1}
    };

    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                const int idx = grid.index(x, y, z);
                auto& neighbors = topology.neighbors.at(static_cast<size_t>(idx));
                for (size_t i = 0; i < neighbors.size(); ++i) {
                    const int nx = x + neighborDirs[i][0];
                    const int ny = y + neighborDirs[i][1];
                    const int nz = z + neighborDirs[i][2];
                    if (grid.inBounds(nx, ny, nz)) {
                        neighbors[i] = grid.index(nx, ny, nz);
                    }
                }

                auto& axialPairs = topology.axialPairs.at(static_cast<size_t>(idx));
                for (size_t axis = 0; axis < 3; ++axis) {
                    const int ax = x + axialDirs[axis][0];
                    const int ay = y + axialDirs[axis][1];
                    const int az = z + axialDirs[axis][2];
                    const int bx = x - axialDirs[axis][0];
                    const int by = y - axialDirs[axis][1];
                    const int bz = z - axialDirs[axis][2];
                    if (grid.inBounds(ax, ay, az)) {
                        axialPairs[axis * 2] = grid.index(ax, ay, az);
                    }
                    if (grid.inBounds(bx, by, bz)) {
                        axialPairs[axis * 2 + 1] = grid.index(bx, by, bz);
                    }
                }
            }
        }
    }
    return topology;
}

std::uint64_t topologyKey(const Grid& grid) {
    std::uint64_t key = static_cast<std::uint64_t>(grid.width());
    key = (key << 20) ^ static_cast<std::uint64_t>(grid.height());
    key = (key << 20) ^ static_cast<std::uint64_t>(grid.depth());
    return key;
}

const RuleTopology& ruleTopologyFor(const Grid& grid) {
    static std::unordered_map<std::uint64_t, RuleTopology> cache;
    const std::uint64_t key = topologyKey(grid);
    if (auto it = cache.find(key); it != cache.end()) {
        return it->second;
    }
    auto inserted = cache.emplace(key, buildRuleTopology(grid));
    return inserted.first->second;
}

bool stateAt(const StateVector* state, int idx) {
    return state != nullptr && state->at(static_cast<size_t>(idx)) != 0U;
}

bool matchesTarget(const Grid& grid, int idx, RuleTarget target, int sinkType, const RuleContext& context) {
    if (idx < 0) {
        return false;
    }

    const Block& block = grid.atIndex(idx);
    switch (target) {
    case RuleTarget::Cell:
        return block.kind == BlockKind::FuelCell &&
               (context.functionalCells == nullptr || stateAt(context.functionalCells, idx));
    case RuleTarget::Moderator:
        return block.kind == BlockKind::Moderator && stateAt(context.activeModerators, idx);
    case RuleTarget::Reflector:
        return block.kind == BlockKind::Reflector && stateAt(context.activeReflectors, idx);
    case RuleTarget::Shield:
        return block.kind == BlockKind::Shield && stateAt(context.functionalShields, idx);
    case RuleTarget::Irradiator:
        return block.kind == BlockKind::Irradiator && stateAt(context.functionalIrradiators, idx);
    case RuleTarget::Conductor:
        return false;
    case RuleTarget::Casing:
        return isCasingLike(block.kind);
    case RuleTarget::AnySink:
        return block.kind == BlockKind::Sink && block.type >= 0 &&
               (context.validSinks == nullptr || stateAt(context.validSinks, idx));
    case RuleTarget::Sink:
        return block.kind == BlockKind::Sink && block.type == sinkType &&
               (context.validSinks == nullptr || stateAt(context.validSinks, idx));
    case RuleTarget::Unknown:
        return false;
    }
    return false;
}

int countAdjacent(const Grid& grid, int idx, const Requirement& requirement, const RuleContext& context,
                  const RuleTopology& topology) {
    int count = 0;
    for (int neighborIdx : topology.neighbors.at(static_cast<size_t>(idx))) {
        if (matchesTarget(grid, neighborIdx, requirement.target, requirement.sinkType, context)) {
            ++count;
        }
    }
    return count;
}

int countAxial(const Grid& grid, int idx, const Requirement& requirement, const RuleContext& context,
               const RuleTopology& topology) {
    int count = 0;
    const auto& pairs = topology.axialPairs.at(static_cast<size_t>(idx));
    for (size_t axis = 0; axis < 3; ++axis) {
        const int a = pairs[axis * 2];
        const int b = pairs[axis * 2 + 1];
        if (matchesTarget(grid, a, requirement.target, requirement.sinkType, context) &&
            matchesTarget(grid, b, requirement.target, requirement.sinkType, context)) {
            count += 2;
        }
    }
    return count;
}

bool satisfied(const Grid& grid, int idx, const Requirement& requirement, const RuleContext& context,
               const RuleTopology& topology) {
    const int count = requirement.axial ? countAxial(grid, idx, requirement, context, topology)
                                        : countAdjacent(grid, idx, requirement, context, topology);
    switch (requirement.countType) {
    case CountType::AtLeast:
        return count >= requirement.count;
    case CountType::Exactly:
        return count == requirement.count;
    case CountType::AtMost:
        return count <= requirement.count;
    }
    return false;
}

bool satisfied(const Grid& grid, int idx, const RuleNode& node, const RuleContext& context,
               const RuleTopology& topology) {
    switch (node.op) {
    case RuleOp::Leaf:
        return satisfied(grid, idx, node.requirement, context, topology);
    case RuleOp::And:
        return std::all_of(node.children.begin(), node.children.end(), [&](const RuleNode& child) {
            return satisfied(grid, idx, child, context, topology);
        });
    case RuleOp::Or:
        return std::any_of(node.children.begin(), node.children.end(), [&](const RuleNode& child) {
            return satisfied(grid, idx, child, context, topology);
        });
    }
    return false;
}

} // namespace

PlacementRule parsePlacementRule(const std::string& text) {
    return {parseNode(text)};
}

const PlacementRule& sinkPlacementRule(int sinkType) {
    static const std::vector<PlacementRule> rules = [] {
        std::vector<PlacementRule> parsedRules;
        parsedRules.reserve(sinkTypes().size());
        for (const SinkType& sink : sinkTypes()) {
            parsedRules.push_back(parsePlacementRule(sink.rule));
        }
        return parsedRules;
    }();
    return rules.at(static_cast<size_t>(sinkType));
}

bool isSinkValidAt(const Grid& grid, const Pos& pos, const RuleContext& context) {
    NCFR_PERF_COUNT(sinkValidityChecks);
    const Block& block = grid.at(pos.x, pos.y, pos.z);
    if (block.kind != BlockKind::Sink || block.type < 0) {
        return false;
    }
    const RuleTopology& topology = ruleTopologyFor(grid);
    return satisfied(grid, grid.index(pos.x, pos.y, pos.z), sinkPlacementRule(block.type).root, context, topology);
}

StateVector evaluateValidSinks(const Grid& grid, const RuleContext& context) {
    NCFR_PERF_COUNT(evaluateValidSinksCalls);
    NCFR_PERF_SCOPE(evaluateValidSinksNs);
    StateVector valid(static_cast<size_t>(grid.volume()), true);
    std::vector<int> sinkIndices;
    sinkIndices.reserve(static_cast<size_t>(grid.volume()));
    for (int idx = 0; idx < grid.volume(); ++idx) {
        const Block& block = grid.atIndex(idx);
        if (block.kind == BlockKind::Sink && block.type >= 0) {
            sinkIndices.push_back(idx);
        }
    }
    if (sinkIndices.empty()) {
        return StateVector(static_cast<size_t>(grid.volume()), false);
    }

    const RuleTopology& topology = ruleTopologyFor(grid);
    bool changed = true;
    for (int pass = 0; pass < 24 && changed; ++pass) {
        NCFR_PERF_COUNT(evaluateValidSinksPasses);
        changed = false;
        RuleContext passContext = context;
        passContext.validSinks = &valid;
        StateVector next(static_cast<size_t>(grid.volume()), false);
        for (int idx : sinkIndices) {
            NCFR_PERF_COUNT(evaluateValidSinksPositions);
            NCFR_PERF_COUNT(sinkValidityChecks);
            const Block& block = grid.atIndex(idx);
            next.at(static_cast<size_t>(idx)) =
                satisfied(grid, idx, sinkPlacementRule(block.type).root, passContext, topology) ? 1U : 0U;
            if (next.at(static_cast<size_t>(idx)) != valid.at(static_cast<size_t>(idx))) {
                changed = true;
            }
        }
        valid.swap(next);
    }
    return valid;
}

} // namespace ncfr
