#pragma once

#include "OptimizerTypes.h"

namespace ncfr::optimizer_detail {

bool samePos(const Pos& lhs, const Pos& rhs);
bool containsDirectionIndex(const std::vector<int>& indices, int index);
Pos offset(const Pos& pos, const Direction& dir, int distance);
Pos sourcePositionForDirection(const Grid& grid, const Pos& fuelPos,
                               const Direction& dir);
std::vector<std::vector<int>> sourceDirectionCombinations(int sourceCount);
std::vector<Dimension> singleFuelSearchDimensions();
bool fuelLineWithinReflectorReach(const FuelLineSpec& line);
std::vector<FuelLineSpec> singleFuelLineOptions(
    const Fuel& fuel, const BuildRequest& request,
    const std::vector<int>& sourceDirections, int direction);
bool placeDirectionalSources(Grid& grid, const BuildRequest& request,
                             const Pos& fuelPos,
                             const std::vector<int>& sourceDirections);
void markProtected(StateVector& protectedPositions, const Grid& grid,
                   const Pos& pos);
bool isFullyReflectiveReflector(const Block& block);
Block sourceLineReplacementBlock(const BuildRequest& request);
void keepSourceLinesOpen(Grid& grid, const BuildRequest& request,
                         const std::vector<int>& sourceDirections);
bool restoreDirectionalFuelLines(
    Grid& grid, const BuildRequest& request,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines);
std::optional<Grid> compactInteriorPlanesPreservingSources(
    const Grid& grid, const BuildRequest& request,
    const std::vector<int>& sourceDirections,
    const std::vector<FuelLineSpec>& fuelLines, int paddingPlanes = 0,
    bool keepConductors = false);
std::optional<Grid> compactInteriorPlanesPreservingSourceTargets(
    const Grid& grid, int paddingPlanes = 0, bool keepConductors = false);

} // namespace ncfr::optimizer_detail
