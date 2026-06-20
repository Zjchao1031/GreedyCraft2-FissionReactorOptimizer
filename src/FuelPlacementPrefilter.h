#pragma once

#include "Grid.h"

#include <vector>

namespace ncfr {

struct BuildRequest;

struct FuelActivationProfile {
    int fuelIndex = -1;
    double criticality = 0.0;
    double intrinsicFlux = 0.0;
    double heat = 0.0;
    bool selfPriming = false;
    int minFullReflectorDirs = 0;
    int minHalfReflectorDirs = 0;
    int minHeavyWaterFuelLinks = 0;
    int minSearchInteriorSize = 5;
};

enum class FuelRelationRejectReason {
    None,
    NoFuelCells,
    MissingFuelData,
    NoSeed,
    FuelNotRunnable
};

struct FuelRelationPrefilterResult {
    bool accepted = false;
    FuelRelationRejectReason reason = FuelRelationRejectReason::None;
    int fuelCells = 0;
    int runningCells = 0;
    int functionalIrradiators = 0;
    double weakestFuelMargin = 0.0;
};

const std::vector<FuelActivationProfile>& fuelActivationProfiles();
const FuelActivationProfile& fuelActivationProfile(int fuelIndex);

FuelRelationPrefilterResult prefilterFuelRelations(const Grid& grid, const BuildRequest& request);
const char* fuelRelationRejectReasonName(FuelRelationRejectReason reason);

} // namespace ncfr
