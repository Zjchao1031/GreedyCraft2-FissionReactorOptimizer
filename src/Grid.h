#pragma once

#include "Data.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ncfr {

enum class BlockKind {
    Empty,
    Casing,
    Controller,
    CellPort,
    IrradiatorPort,
    VentIn,
    VentOut,
    Source,
    FuelCell,
    Moderator,
    Reflector,
    Shield,
    Irradiator,
    Conductor,
    Sink
};

struct Block {
    BlockKind kind = BlockKind::Empty;
    int type = -1;
};

struct Pos {
    int x = 0;
    int y = 0;
    int z = 0;
};

enum class FuelCellPortRole {
    Input = 0,
    Output = 1
};

class Grid {
public:
    Grid(int internalA, int internalB, int internalC);

    int internalA() const { return a_; }
    int internalB() const { return b_; }
    int internalC() const { return c_; }
    int width() const { return a_ + 2; }
    int height() const { return b_ + 2; }
    int depth() const { return c_ + 2; }
    int volume() const { return width() * height() * depth(); }

    bool inBounds(int x, int y, int z) const;
    bool isBoundary(int x, int y, int z) const;
    bool isInterior(int x, int y, int z) const;
    int index(int x, int y, int z) const;

    const Block& at(int x, int y, int z) const;
    Block& at(int x, int y, int z);
    const Block& atIndex(int idx) const { return blocks_[idx]; }
    Block& atIndex(int idx) { return blocks_[idx]; }

    std::uint64_t fingerprint() const;
    template <typename Fn>
    void forEachNeighbor6(const Pos& p, Fn&& fn) const {
        static constexpr int dirs[6][3] = {
            {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
        };
        for (const auto& d : dirs) {
            Pos n{p.x + d[0], p.y + d[1], p.z + d[2]};
            if (inBounds(n.x, n.y, n.z)) {
                fn(n);
            }
        }
    }
    std::vector<Pos> interiorPositions() const;
    std::vector<Pos> neighbors6(const Pos& p) const;

private:
    int a_;
    int b_;
    int c_;
    std::vector<Block> blocks_;
};

Grid makeShell(int a, int b, int c);
int fuelCellPortType(int fuelIndex, FuelCellPortRole role);
int fuelCellPortFuelIndex(int type);
FuelCellPortRole fuelCellPortRole(int type);
const char* fuelCellPortRoleNameZh(FuelCellPortRole role);
int irradiatorPortType(FuelCellPortRole role);
FuelCellPortRole irradiatorPortRole(int type);
bool isCasingLike(BlockKind kind);
bool isFunctionalInterior(BlockKind kind);
std::string blockDisplayName(const Block& block);

} // namespace ncfr
