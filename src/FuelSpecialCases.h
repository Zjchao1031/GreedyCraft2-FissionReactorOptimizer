#pragma once

#include "Data.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <string_view>

namespace ncfr {

inline bool fuelEnglishNameEquals(const Fuel& fuel, std::string_view name) {
    return fuel.nameEn.size() == name.size() &&
           std::equal(fuel.nameEn.begin(), fuel.nameEn.end(), name.begin());
}

template <std::size_t N>
inline bool fuelEnglishNameIn(const Fuel& fuel, const std::array<std::string_view, N>& names) {
    return std::any_of(names.begin(), names.end(), [&](std::string_view name) {
        return fuelEnglishNameEquals(fuel, name);
    });
}

inline bool usesSpecialManaDustCornerSinks(const Fuel& fuel) {
    static constexpr std::array<std::string_view, 10> kFuelNames{
        "HEE-254 Zirconium Alloy Fuel Pellet",
        "IPCf-249 TRISO Fuel Pebble",
        "IPCf-249 Oxide Fuel Pellet",
        "HECf-253 TRISO Fuel Pebble",
        "HECf-253 Oxide Fuel Pellet",
        "UECf-249 Zirconium Alloy Fuel Pellet",
        "WGE-254 Nitride Fuel Pellet",
        "MTRISO-294 Fuel Pebble",
        "MOX-294 Fuel Pellet",
        "XEE-254 Zirconium Alloy Fuel Pellet",
    };
    return fuelEnglishNameIn(fuel, kFuelNames);
}

inline bool blocksNormalSingleFuelGeneration(const Fuel& fuel) {
    static constexpr std::array<std::string_view, 16> kFuelNames{
        "IPE-254 Zirconium Alloy Fuel Pellet",
        "WGE-254 Zirconium Alloy Fuel Pellet",
        "IPE-254 TRISO Fuel Pebble",
        "IPE-254 Oxide Fuel Pellet",
        "UEE-254 Zirconium Alloy Fuel Pellet",
        "IPCf-249 Zirconium Alloy Fuel Pellet",
        "HECf-253 Zirconium Alloy Fuel Pellet",
        "WGE-254 TRISO Fuel Pebble",
        "WGE-254 Oxide Fuel Pellet",
        "MZA-294 Fuel Pellet",
        "WGCf-249 Zirconium Alloy Fuel Pellet",
        "SEE-254 Zirconium Alloy Fuel Pellet",
        "MZA-258 Fuel Pellet",
        "IPE-254 Nitride Fuel Pellet",
        "UEE-254 TRISO Fuel Pebble",
        "UEE-254 Oxide Fuel Pellet",
    };
    return fuelEnglishNameIn(fuel, kFuelNames);
}

} // namespace ncfr
