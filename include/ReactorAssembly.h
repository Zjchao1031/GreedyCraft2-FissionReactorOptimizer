#pragma once

#include "Grid.h"

#include <vector>

namespace ncfr {

std::vector<int> fuelIndicesInGrid(const Grid& grid);
std::vector<int> uniqueFuelIndices(const std::vector<int>& fuelIndices);
std::vector<Pos> fuelCellPortPositions(const Grid& grid);

bool rebuildFuelCellPorts(Grid& grid, const std::vector<int>& fuelIndices);
void addFuelCellPorts(Grid& grid, const std::vector<int>& fuelIndices);
void addIrradiatorPort(Grid& grid);
bool hasRequiredBoundaryParts(const Grid& grid,
                              const std::vector<int>& fuelIndices);

} // namespace ncfr
