#include "ReactorAssembly.h"

#include "Data.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace ncfr {
namespace {

void clearFuelCellPorts(Grid& grid) {
    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                Block& block = grid.at(x, y, z);
                if (grid.isBoundary(x, y, z) &&
                    block.kind == BlockKind::CellPort) {
                    block = {BlockKind::Casing, -1};
                }
            }
        }
    }
}

} // namespace

std::vector<int> fuelIndicesInGrid(const Grid& grid) {
    std::vector<int> indices;
    for (const Pos& pos : grid.interiorPositions()) {
        const Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == BlockKind::FuelCell && block.type >= 0) {
            indices.push_back(block.type);
        }
    }
    return indices;
}

std::vector<int> uniqueFuelIndices(const std::vector<int>& fuelIndices) {
    std::vector<int> unique;
    unique.reserve(fuelIndices.size());
    for (int fuelIndex : fuelIndices) {
        if (std::find(unique.begin(), unique.end(), fuelIndex) ==
            unique.end()) {
            unique.push_back(fuelIndex);
        }
    }
    return unique;
}

std::vector<Pos> fuelCellPortPositions(const Grid& grid) {
    std::vector<Pos> positions;
    positions.reserve(static_cast<size_t>(grid.volume()));
    const auto appendIfCasing =
        [&](int x, int y, int z) {
            if (grid.isBoundary(x, y, z) &&
                grid.at(x, y, z).kind == BlockKind::Casing) {
                positions.push_back({x, y, z});
            }
        };

    for (int z = 1; z < grid.depth() - 1; ++z) {
        for (int y = 1; y < grid.height() - 1; ++y) {
            appendIfCasing(0, y, z);
            appendIfCasing(grid.width() - 1, y, z);
        }
    }
    for (int z = 1; z < grid.depth() - 1; ++z) {
        for (int x = 1; x < grid.width() - 1; ++x) {
            appendIfCasing(x, 0, z);
            appendIfCasing(x, grid.height() - 1, z);
        }
    }
    for (int y = 1; y < grid.height() - 1; ++y) {
        for (int x = 1; x < grid.width() - 1; ++x) {
            appendIfCasing(x, y, 0);
            appendIfCasing(x, y, grid.depth() - 1);
        }
    }
    return positions;
}

bool rebuildFuelCellPorts(Grid& grid, const std::vector<int>& fuelIndices) {
    Grid rebuilt = grid;
    clearFuelCellPorts(rebuilt);

    const std::vector<int> unique = uniqueFuelIndices(fuelIndices);
    const std::vector<Pos> positions = fuelCellPortPositions(rebuilt);
    if (positions.size() < unique.size() * 2) {
        return false;
    }

    size_t positionIndex = 0;
    for (int fuelIndex : unique) {
        const Pos inputPos = positions.at(positionIndex++);
        rebuilt.at(inputPos.x, inputPos.y, inputPos.z) = {
            BlockKind::CellPort,
            fuelCellPortType(fuelIndex, FuelCellPortRole::Input),
        };

        const Pos outputPos = positions.at(positionIndex++);
        rebuilt.at(outputPos.x, outputPos.y, outputPos.z) = {
            BlockKind::CellPort,
            fuelCellPortType(fuelIndex, FuelCellPortRole::Output),
        };
    }

    grid = std::move(rebuilt);
    return true;
}

void addFuelCellPorts(Grid& grid, const std::vector<int>& fuelIndices) {
    if (!rebuildFuelCellPorts(grid, fuelIndices)) {
        throw std::runtime_error(
            "外壳空间不足，无法为每种燃料放置输入/输出燃料单元端口。");
    }
}

void addIrradiatorPort(Grid& grid) {
    bool hasIrradiator = false;
    int portCount = 0;
    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                const Block& block = grid.at(x, y, z);
                hasIrradiator =
                    hasIrradiator || block.kind == BlockKind::Irradiator;
                if (block.kind == BlockKind::IrradiatorPort) {
                    ++portCount;
                }
            }
        }
    }
    if (!hasIrradiator || portCount >= 2) {
        return;
    }

    for (const Pos& pos : fuelCellPortPositions(grid)) {
        if (portCount >= 2) {
            return;
        }
        const FuelCellPortRole role =
            portCount == 0 ? FuelCellPortRole::Input
                           : FuelCellPortRole::Output;
        grid.at(pos.x, pos.y, pos.z) = {
            BlockKind::IrradiatorPort,
            irradiatorPortType(role),
        };
        ++portCount;
    }
    throw std::runtime_error(
        "外壳空间不足，无法放置输入/输出辐照器端口。");
}

bool hasRequiredBoundaryParts(const Grid& grid,
                              const std::vector<int>& fuelIndices) {
    int controllers = 0;
    int inputVents = 0;
    int outputVents = 0;
    bool hasIrradiator = false;
    std::vector<bool> fuelInputs(fuels().size(), false);
    std::vector<bool> fuelOutputs(fuels().size(), false);
    bool irradiatorInput = false;
    bool irradiatorOutput = false;

    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                const Block& block = grid.at(x, y, z);
                if (grid.isInterior(x, y, z)) {
                    hasIrradiator =
                        hasIrradiator ||
                        block.kind == BlockKind::Irradiator;
                    continue;
                }
                switch (block.kind) {
                case BlockKind::Controller:
                    ++controllers;
                    break;
                case BlockKind::VentIn:
                    ++inputVents;
                    break;
                case BlockKind::VentOut:
                    ++outputVents;
                    break;
                case BlockKind::CellPort: {
                    const int fuelIndex =
                        fuelCellPortFuelIndex(block.type);
                    if (fuelIndex >= 0 &&
                        fuelIndex < static_cast<int>(fuels().size())) {
                        if (fuelCellPortRole(block.type) ==
                            FuelCellPortRole::Input) {
                            fuelInputs.at(
                                static_cast<size_t>(fuelIndex)) = true;
                        } else {
                            fuelOutputs.at(
                                static_cast<size_t>(fuelIndex)) = true;
                        }
                    }
                    break;
                }
                case BlockKind::IrradiatorPort:
                    if (irradiatorPortRole(block.type) ==
                        FuelCellPortRole::Input) {
                        irradiatorInput = true;
                    } else {
                        irradiatorOutput = true;
                    }
                    break;
                default:
                    break;
                }
            }
        }
    }

    if (controllers != 1 || inputVents != 1 || outputVents != 1) {
        return false;
    }
    for (int fuelIndex : uniqueFuelIndices(fuelIndices)) {
        if (fuelIndex < 0 ||
            fuelIndex >= static_cast<int>(fuels().size()) ||
            !fuelInputs.at(static_cast<size_t>(fuelIndex)) ||
            !fuelOutputs.at(static_cast<size_t>(fuelIndex))) {
            return false;
        }
    }
    return !hasIrradiator || (irradiatorInput && irradiatorOutput);
}

} // namespace ncfr
