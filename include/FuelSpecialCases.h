#pragma once

#include "Data.h"

#include <cstddef>

namespace ncfr {

inline constexpr double kEndStoneFallbackFuelHeatThreshold = 2190.0;
inline constexpr double kEndStoneFallbackFuelHeatLimit = 2515.0;
inline constexpr double kCarobbiiteFallbackFuelHeatLimit = 3075.0;
inline constexpr double kManaDustFallbackFuelHeatLimit = 3715.0;
inline constexpr double kNormalSingleFuelHeatLimit = kManaDustFallbackFuelHeatLimit;
inline constexpr double kIrradiatorTotalFuelHeatLimit = 12905.0;
inline constexpr long long kEndStoneFallbackCoolingCapacity = 325;
inline constexpr long long kCarobbiiteFallbackCoolingCapacity = 560;
inline constexpr long long kEndStoneCarobbiiteFallbackCoolingCapacity =
    kEndStoneFallbackCoolingCapacity + kCarobbiiteFallbackCoolingCapacity;
inline constexpr long long kManaDustFallbackCoolingCapacity = 640;
inline constexpr long long kCombinedHighHeatFallbackCoolingCapacity =
    kEndStoneCarobbiiteFallbackCoolingCapacity +
    kManaDustFallbackCoolingCapacity;
inline constexpr long long kMixedFuelStageCoolingTarget = 2190;

struct MixedFuelSpecialCoolingLimits {
    long long endStoneDeficitLimit = 0;
    long long carobbiiteDeficitLimit = 0;
    long long manaDustDeficitLimit = 0;
};

inline MixedFuelSpecialCoolingLimits mixedFuelSpecialCoolingLimits(
    std::size_t fuelContextCount) {
    const long long fuelCount =
        static_cast<long long>(fuelContextCount);
    const long long endStoneCapacity =
        fuelCount * kEndStoneFallbackCoolingCapacity;
    const long long carobbiiteCapacity =
        endStoneCapacity +
        fuelCount * kCarobbiiteFallbackCoolingCapacity;
    return {
        endStoneCapacity,
        carobbiiteCapacity,
        carobbiiteCapacity + kManaDustFallbackCoolingCapacity,
    };
}

inline bool usesEndStoneOnlyReflectorCooling(const Fuel& fuel) {
    return fuel.heat > kEndStoneFallbackFuelHeatThreshold &&
           fuel.heat <= kEndStoneFallbackFuelHeatLimit;
}

inline bool usesCarobbiiteReflectorCooling(const Fuel& fuel) {
    return fuel.heat > kEndStoneFallbackFuelHeatLimit &&
           fuel.heat <= kCarobbiiteFallbackFuelHeatLimit;
}

inline bool usesEndStoneReflectorCooling(const Fuel& fuel) {
    return usesEndStoneOnlyReflectorCooling(fuel) ||
           usesCarobbiiteReflectorCooling(fuel);
}

inline bool hasEndStoneFallbackCoolingDeficit(long long deficit) {
    return deficit > 0 && deficit <= kEndStoneFallbackCoolingCapacity;
}

inline bool hasEndStoneCarobbiiteFallbackCoolingDeficit(long long deficit) {
    return deficit > 0 && deficit <= kEndStoneCarobbiiteFallbackCoolingCapacity;
}

inline bool hasManaDustFallbackCoolingDeficit(long long deficit) {
    return deficit > 0 && deficit <= kManaDustFallbackCoolingCapacity;
}

inline bool hasCombinedHighHeatFallbackCoolingDeficit(long long deficit) {
    return deficit > 0 &&
           deficit <= kCombinedHighHeatFallbackCoolingCapacity;
}

inline bool usesSpecialManaDustCornerSinks(const Fuel& fuel) {
    return fuel.heat > kCarobbiiteFallbackFuelHeatLimit &&
           fuel.heat <= kManaDustFallbackFuelHeatLimit;
}

inline bool blocksNormalSingleFuelGeneration(const Fuel& fuel) {
    return fuel.heat > kNormalSingleFuelHeatLimit;
}

inline bool blocksIrradiatorGeneration(long long rawHeating) {
    return rawHeating > kIrradiatorTotalFuelHeatLimit;
}

} // namespace ncfr
