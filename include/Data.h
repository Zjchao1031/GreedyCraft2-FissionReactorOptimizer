#pragma once

#include <string>
#include <vector>

namespace ncfr {

struct Fuel {
    std::string familyZh;
    std::string formZh;
    std::string nameZh;
    double heat;
    double criticality;
    double intrinsicFlux;
    bool selfPriming;
};

struct SinkType {
    int index;
    std::string ruleId;
    std::string sourceName;
    std::string nameZh;
    int cooling;
    std::string rule;
};

struct ModeratorType {
    std::string nameZh;
    std::string nameEn;
    int fluxFactor;
};

struct ReflectorType {
    std::string registryName;
    std::string nameZh;
    std::string nameEn;
    double reflectivity;
};

struct ShieldType {
    std::string registryName;
    std::string nameZh;
    std::string nameEn;
    double heatPerFlux;
};

struct IrradiatorRecipeType {
    std::string inputName;
    std::string outputName;
    std::string nameZh;
    std::string nameEn;
    double heatPerFlux;
};

const std::vector<Fuel>& fuels();
const std::vector<SinkType>& sinkTypes();
const std::vector<ModeratorType>& moderatorTypes();
const std::vector<ReflectorType>& reflectorTypes();
const std::vector<ShieldType>& shieldTypes();
const std::vector<IrradiatorRecipeType>& irradiatorRecipeTypes();
int defaultIrradiatorRecipeIndex();

} // namespace ncfr
