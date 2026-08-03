#pragma once

#include "Data.h"
#include "Grid.h"

#include <array>
#include <vector>

namespace ncfr {

inline constexpr std::array<Pos, 6> kNeutronLineDirections = {{
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1},
}};

enum class NeutronLineEndpoint {
    None,
    FuelCell,
    Irradiator,
    Reflector,
};

struct NeutronShieldFlux {
    int index = -1;
    double incomingFlux = 0.0;
};

struct NeutronLineActivity {
    std::vector<int> moderatorIndices;
    std::vector<NeutronShieldFlux> shields;
};

struct NeutronLineResult {
    NeutronLineEndpoint endpoint = NeutronLineEndpoint::None;
    int targetIndex = -1;
    double flux = 0.0;
};

NeutronLineResult traceNeutronLine(const Grid& grid, const Fuel& fuel,
                                   const Pos& from, const Pos& direction,
                                   NeutronLineActivity* activity = nullptr);

} // namespace ncfr
