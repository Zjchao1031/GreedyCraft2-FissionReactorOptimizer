#include "Grid.h"

#include <cstdint>
#include <stdexcept>

namespace ncfr {
namespace {

constexpr int kFuelCellPortRoleCount = 2;

} // namespace

Grid::Grid(int internalA, int internalB, int internalC)
    : a_(internalA), b_(internalB), c_(internalC),
      blocks_(static_cast<size_t>((internalA + 2) * (internalB + 2) * (internalC + 2))) {
    if (internalA <= 0 || internalB <= 0 || internalC <= 0) {
        throw std::invalid_argument("尺寸必须为正整数");
    }
}

bool Grid::inBounds(int x, int y, int z) const {
    return x >= 0 && y >= 0 && z >= 0 && x < width() && y < height() && z < depth();
}

bool Grid::isBoundary(int x, int y, int z) const {
    return inBounds(x, y, z) && (x == 0 || y == 0 || z == 0 || x == width() - 1 || y == height() - 1 || z == depth() - 1);
}

bool Grid::isInterior(int x, int y, int z) const {
    return inBounds(x, y, z) && !isBoundary(x, y, z);
}

int Grid::index(int x, int y, int z) const {
    return (z * height() + y) * width() + x;
}

const Block& Grid::at(int x, int y, int z) const {
    return blocks_[index(x, y, z)];
}

Block& Grid::at(int x, int y, int z) {
    return blocks_[index(x, y, z)];
}

std::uint64_t Grid::fingerprint() const {
    std::uint64_t hash = 1469598103934665603ULL;
    auto mix = [&hash](std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ULL;
    };

    mix(static_cast<std::uint64_t>(a_));
    mix(static_cast<std::uint64_t>(b_));
    mix(static_cast<std::uint64_t>(c_));
    for (const Block& block : blocks_) {
        mix(static_cast<std::uint64_t>(block.kind));
        mix(static_cast<std::uint64_t>(static_cast<std::uint32_t>(block.type)));
    }
    return hash;
}

std::vector<Pos> Grid::interiorPositions() const {
    std::vector<Pos> positions;
    positions.reserve(static_cast<size_t>(a_ * b_ * c_));
    for (int z = 1; z <= c_; ++z) {
        for (int y = 1; y <= b_; ++y) {
            for (int x = 1; x <= a_; ++x) {
                positions.push_back({x, y, z});
            }
        }
    }
    return positions;
}

std::vector<Pos> Grid::neighbors6(const Pos& p) const {
    static constexpr int dirs[6][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    std::vector<Pos> result;
    result.reserve(6);
    for (const auto& d : dirs) {
        Pos n{p.x + d[0], p.y + d[1], p.z + d[2]};
        if (inBounds(n.x, n.y, n.z)) {
            result.push_back(n);
        }
    }
    return result;
}

Grid makeShell(int a, int b, int c) {
    Grid grid(a, b, c);
    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                if (grid.isBoundary(x, y, z)) {
                    grid.at(x, y, z) = {BlockKind::Casing, -1};
                }
            }
        }
    }

    grid.at(0, 1, 1) = {BlockKind::Controller, -1};
    if (grid.width() > 2) {
        grid.at(1, 0, 1) = {BlockKind::VentIn, -1};
    }
    if (grid.width() > 2 && grid.height() > 2) {
        grid.at(grid.width() - 2, grid.height() - 1, 1) = {BlockKind::VentOut, -1};
    }
    return grid;
}

int fuelCellPortType(int fuelIndex, FuelCellPortRole role) {
    return fuelIndex * kFuelCellPortRoleCount + static_cast<int>(role);
}

int fuelCellPortFuelIndex(int type) {
    if (type < 0) {
        return -1;
    }
    return type / kFuelCellPortRoleCount;
}

FuelCellPortRole fuelCellPortRole(int type) {
    return type >= 0 && type % kFuelCellPortRoleCount == static_cast<int>(FuelCellPortRole::Output)
        ? FuelCellPortRole::Output
        : FuelCellPortRole::Input;
}

const char* fuelCellPortRoleNameZh(FuelCellPortRole role) {
    return role == FuelCellPortRole::Output ? "输出" : "输入";
}

int irradiatorPortType(FuelCellPortRole role) {
    return static_cast<int>(role);
}

FuelCellPortRole irradiatorPortRole(int type) {
    return type == static_cast<int>(FuelCellPortRole::Output)
        ? FuelCellPortRole::Output
        : FuelCellPortRole::Input;
}

bool isCasingLike(BlockKind kind) {
    return kind == BlockKind::Casing || kind == BlockKind::Controller || kind == BlockKind::CellPort ||
           kind == BlockKind::IrradiatorPort || kind == BlockKind::VentIn || kind == BlockKind::VentOut ||
           kind == BlockKind::Source;
}

bool isFunctionalInterior(BlockKind kind) {
    return kind == BlockKind::FuelCell || kind == BlockKind::Moderator || kind == BlockKind::Reflector ||
           kind == BlockKind::Shield || kind == BlockKind::Irradiator || kind == BlockKind::Sink;
}

std::string blockDisplayName(const Block& block) {
    switch (block.kind) {
    case BlockKind::Empty:
        return "空位";
    case BlockKind::Casing:
        return "裂变反应堆外壳";
    case BlockKind::Controller:
        return "固态燃料裂变控制器";
    case BlockKind::CellPort:
        if (block.type >= 0) {
            const int fuelIndex = fuelCellPortFuelIndex(block.type);
            if (fuelIndex >= 0 && fuelIndex < static_cast<int>(fuels().size())) {
                return fuels().at(static_cast<size_t>(fuelIndex)).nameZh + "燃料单元端口(" +
                       fuelCellPortRoleNameZh(fuelCellPortRole(block.type)) + ")";
            }
        }
        return "裂变燃料单元端口";
    case BlockKind::IrradiatorPort:
        if (block.type >= 0) {
            return std::string("裂变辐照端口（") + fuelCellPortRoleNameZh(irradiatorPortRole(block.type)) + "）";
        }
        return "裂变辐照端口";
    case BlockKind::VentIn:
        return "裂变通风口(输入)";
    case BlockKind::VentOut:
        return "裂变通风口(输出)";
    case BlockKind::Source:
        if (block.type < 0) {
            return "任意中子源";
        }
        return sourceTypes().at(static_cast<size_t>(block.type)).nameZh;
    case BlockKind::FuelCell:
        if (block.type >= 0) {
            return fuels().at(static_cast<size_t>(block.type)).nameZh;
        }
        return "裂变燃料单元";
    case BlockKind::Moderator:
        return moderatorTypes().at(static_cast<size_t>(block.type)).nameZh;
    case BlockKind::Reflector:
        return reflectorTypes().at(static_cast<size_t>(block.type)).nameZh;
    case BlockKind::Shield:
        return shieldTypes().at(static_cast<size_t>(block.type)).nameZh;
    case BlockKind::Irradiator:
        return "裂变中子辐照器";
    case BlockKind::Sink:
        return sinkTypes().at(static_cast<size_t>(block.type)).nameZh;
    }
    return "未知";
}

} // namespace ncfr
