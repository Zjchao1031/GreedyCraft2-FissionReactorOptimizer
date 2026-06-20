#pragma once

#include <string>
#include <vector>

namespace ncfr {

struct Fuel {
    std::string familyZh;
    std::string familyEn;
    std::string code;
    std::string formZh;
    std::string nameZh;
    std::string nameEn;
    double time;
    double heat;
    double efficiency;
    double criticality;
    double intrinsicFlux;
    double decayFactor;
    bool selfPriming;
};

struct SinkType {
    int index;
    std::string ruleId;
    std::string sourceName;
    std::string nameZh;
    std::string nameEn;
    int cooling;
    std::string rule;
};

struct ModeratorType {
    std::string registryName;
    std::string nameZh;
    std::string nameEn;
    int fluxFactor;
    double efficiency;
};

struct ReflectorType {
    std::string registryName;
    std::string nameZh;
    std::string nameEn;
    double efficiency;
    double reflectivity;
};

struct SourceType {
    std::string registryName;
    std::string nameZh;
    std::string nameEn;
    double efficiency;
};

struct ShieldType {
    std::string registryName;
    std::string nameZh;
    std::string nameEn;
    double heatPerFlux;
    double efficiency;
};

struct IrradiatorRecipeType {
    std::string inputName;
    std::string outputName;
    std::string nameZh;
    std::string nameEn;
    double heatPerFlux;
    double efficiency;
};

const std::vector<Fuel>& fuels();
const std::vector<SinkType>& sinkTypes();
const std::vector<ModeratorType>& moderatorTypes();
const std::vector<ReflectorType>& reflectorTypes();
const std::vector<SourceType>& sourceTypes();
const std::vector<ShieldType>& shieldTypes();
const std::vector<IrradiatorRecipeType>& irradiatorRecipeTypes();
int defaultIrradiatorRecipeIndex();

} // namespace ncfr
