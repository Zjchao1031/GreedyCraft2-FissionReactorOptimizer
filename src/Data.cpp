// Generated from NuclearCraft_FissionReactor_SourceData.xlsx. Do not add values from other sources.
#include "Data.h"

namespace ncfr {

const std::vector<Fuel>& fuels() {
    static const std::vector<Fuel> data = {
        {"钍", "Thorium", "TBU-TRISO", "TRISO", "钍增殖铀 TRISO 燃料颗粒", "TBU TRISO Fuel Pebble", 14400, 40, 1.25, 199, 10, 0.04, false},
        {"钍", "Thorium", "TBU-OX", "氧化物", "钍增殖铀 氧化物燃料丸", "TBU Oxide Fuel Pellet", 14400, 40, 1.25, 234, 0, 0.04, false},
        {"钍", "Thorium", "TBU-NI", "氮化物", "钍增殖铀 氮化物燃料丸", "TBU Nitride Fuel Pellet", 18000, 32, 1.25, 293, 0, 0.04, false},
        {"钍", "Thorium", "TBU-ZA", "锆合金", "钍增殖铀-锆合金燃料丸", "TBU-Zirconium Alloy Fuel Pellet", 11520, 50, 1.25, 199, 0, 0.04, false},
        {"钍", "Thorium", "TBU-F4", "熔盐 (F4)", "熔融钍增殖铀 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of TBU Fluoride Fuel", 18000, 32, 1.25, 234, 0, 0.04, false},
        {"铀", "Uranium", "LEU-233-TRISO", "TRISO", "低浓缩铀-233 TRISO 燃料颗粒", "LEU-233 TRISO Fuel Pebble", 2666, 216, 1.1, 66, 10, 0.065, false},
        {"铀", "Uranium", "LEU-233-OX", "氧化物", "低浓缩铀-233 氧化物燃料丸", "LEU-233 Oxide Fuel Pellet", 2666, 216, 1.1, 78, 0, 0.065, false},
        {"铀", "Uranium", "LEU-233-NI", "氮化物", "低浓缩铀-233 氮化物燃料丸", "LEU-233 Nitride Fuel Pellet", 3348, 172, 1.1, 98, 0, 0.065, false},
        {"铀", "Uranium", "LEU-233-ZA", "锆合金", "低浓缩铀-233-锆合金燃料丸", "LEU-233-Zirconium Alloy Fuel Pellet", 2134, 270, 1.1, 66, 0, 0.065, false},
        {"铀", "Uranium", "LEU-233-F4", "熔盐 (F4)", "熔融低浓缩铀-233 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LEU-233 Fluoride Fuel", 3348, 172, 1.1, 78, 0, 0.065, false},
        {"铀", "Uranium", "HEU-233-TRISO", "TRISO", "高浓缩铀-233 TRISO 燃料颗粒", "HEU-233 TRISO Fuel Pebble", 2666, 648, 1.15, 33, 10, 0.065, false},
        {"铀", "Uranium", "HEU-233-OX", "氧化物", "高浓缩铀-233 氧化物燃料丸", "HEU-233 Oxide Fuel Pellet", 2666, 648, 1.15, 39, 0, 0.065, false},
        {"铀", "Uranium", "HEU-233-NI", "氮化物", "高浓缩铀-233 氮化物燃料丸", "HEU-233 Nitride Fuel Pellet", 3348, 516, 1.15, 49, 0, 0.065, false},
        {"铀", "Uranium", "HEU-233-ZA", "锆合金", "高浓缩铀-233-锆合金燃料丸", "HEU-233-Zirconium Alloy Fuel Pellet", 2134, 810, 1.15, 33, 0, 0.065, false},
        {"铀", "Uranium", "HEU-233-F4", "熔盐 (F4)", "熔融高浓缩铀-233 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HEU-233 Fluoride Fuel", 3348, 516, 1.15, 39, 0, 0.065, false},
        {"铀", "Uranium", "LEU-235-TRISO", "TRISO", "低浓缩铀-235 TRISO 燃料颗粒", "LEU-235 TRISO Fuel Pebble", 4800, 120, 1, 87, 10, 0.065, false},
        {"铀", "Uranium", "LEU-235-OX", "氧化物", "低浓缩铀-235 氧化物燃料丸", "LEU-235 Oxide Fuel Pellet", 4800, 120, 1, 102, 0, 0.065, false},
        {"铀", "Uranium", "LEU-235-NI", "氮化物", "低浓缩铀-235 氮化物燃料丸", "LEU-235 Nitride Fuel Pellet", 6000, 96, 1, 128, 0, 0.065, false},
        {"铀", "Uranium", "LEU-235-ZA", "锆合金", "低浓缩铀-235-锆合金燃料丸", "LEU-235-Zirconium Alloy Fuel Pellet", 3840, 150, 1, 87, 0, 0.065, false},
        {"铀", "Uranium", "LEU-235-F4", "熔盐 (F4)", "熔融低浓缩铀-235 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LEU-235 Fluoride Fuel", 6000, 96, 1, 102, 0, 0.065, false},
        {"铀", "Uranium", "HEU-235-TRISO", "TRISO", "高浓缩铀-235 TRISO 燃料颗粒", "HEU-235 TRISO Fuel Pebble", 4800, 360, 1.05, 43, 10, 0.065, false},
        {"铀", "Uranium", "HEU-235-OX", "氧化物", "高浓缩铀-235 氧化物燃料丸", "HEU-235 Oxide Fuel Pellet", 4800, 360, 1.05, 51, 0, 0.065, false},
        {"铀", "Uranium", "HEU-235-NI", "氮化物", "高浓缩铀-235 氮化物燃料丸", "HEU-235 Nitride Fuel Pellet", 6000, 288, 1.05, 64, 0, 0.065, false},
        {"铀", "Uranium", "HEU-235-ZA", "锆合金", "高浓缩铀-235-锆合金燃料丸", "HEU-235-Zirconium Alloy Fuel Pellet", 3840, 450, 1.05, 43, 0, 0.065, false},
        {"铀", "Uranium", "HEU-235-F4", "熔盐 (F4)", "熔融高浓缩铀-235 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HEU-235 Fluoride Fuel", 6000, 288, 1.05, 51, 0, 0.065, false},
        {"镎", "Neptunium", "LEN-236-TRISO", "TRISO", "低浓缩镎-236 TRISO 燃料颗粒", "LEN-236 TRISO Fuel Pebble", 1972, 292, 1.1, 60, 10, 0.07, false},
        {"镎", "Neptunium", "LEN-236-OX", "氧化物", "低浓缩镎-236 氧化物燃料丸", "LEN-236 Oxide Fuel Pellet", 1972, 292, 1.1, 70, 0, 0.07, false},
        {"镎", "Neptunium", "LEN-236-NI", "氮化物", "低浓缩镎-236 氮化物燃料丸", "LEN-236 Nitride Fuel Pellet", 2462, 234, 1.1, 88, 0, 0.07, false},
        {"镎", "Neptunium", "LEN-236-ZA", "锆合金", "低浓缩镎-236-锆合金燃料丸", "LEN-236-Zirconium Alloy Fuel Pellet", 1574, 366, 1.1, 60, 0, 0.07, false},
        {"镎", "Neptunium", "LEN-236-F4", "熔盐 (F4)", "熔融低浓缩镎-236 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LEN-236 Fluoride Fuel", 2462, 234, 1.1, 70, 0, 0.07, false},
        {"镎", "Neptunium", "HEN-236-TRISO", "TRISO", "高浓缩镎-236 TRISO 燃料颗粒", "HEN-236 TRISO Fuel Pebble", 1972, 876, 1.15, 30, 10, 0.07, false},
        {"镎", "Neptunium", "HEN-236-OX", "氧化物", "高浓缩镎-236 氧化物燃料丸", "HEN-236 Oxide Fuel Pellet", 1972, 876, 1.15, 35, 0, 0.07, false},
        {"镎", "Neptunium", "HEN-236-NI", "氮化物", "高浓缩镎-236 氮化物燃料丸", "HEN-236 Nitride Fuel Pellet", 2462, 702, 1.15, 44, 0, 0.07, false},
        {"镎", "Neptunium", "HEN-236-ZA", "锆合金", "高浓缩镎-236-锆合金燃料丸", "HEN-236-Zirconium Alloy Fuel Pellet", 1574, 1098, 1.15, 30, 0, 0.07, false},
        {"镎", "Neptunium", "HEN-236-F4", "熔盐 (F4)", "熔融高浓缩镎-236 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HEN-236 Fluoride Fuel", 2462, 702, 1.15, 35, 0, 0.07, false},
        {"钚", "Plutonium", "LEP-239-TRISO", "TRISO", "低浓缩钚-239 TRISO 燃料颗粒", "LEP-239 TRISO Fuel Pebble", 4572, 126, 1.2, 84, 10, 0.075, false},
        {"钚", "Plutonium", "LEP-239-OX", "氧化物", "低浓缩钚-239 氧化物燃料丸", "LEP-239 Oxide Fuel Pellet", 4572, 126, 1.2, 99, 0, 0.075, false},
        {"钚", "Plutonium", "LEP-239-NI", "氮化物", "低浓缩钚-239 氮化物燃料丸", "LEP-239 Nitride Fuel Pellet", 5760, 100, 1.2, 124, 0, 0.075, false},
        {"钚", "Plutonium", "LEP-239-ZA", "锆合金", "低浓缩钚-239-锆合金燃料丸", "LEP-239-Zirconium Alloy Fuel Pellet", 3646, 158, 1.2, 84, 0, 0.075, false},
        {"钚", "Plutonium", "LEP-239-F4", "熔盐 (F4)", "熔融低浓缩钚-239 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LEP-239 Fluoride Fuel", 5760, 100, 1.2, 99, 0, 0.075, false},
        {"钚", "Plutonium", "HEP-239-TRISO", "TRISO", "高浓缩钚-239 TRISO 燃料颗粒", "HEP-239 TRISO Fuel Pebble", 4572, 378, 1.25, 42, 10, 0.075, false},
        {"钚", "Plutonium", "HEP-239-OX", "氧化物", "高浓缩钚-239 氧化物燃料丸", "HEP-239 Oxide Fuel Pellet", 4572, 378, 1.25, 49, 0, 0.075, false},
        {"钚", "Plutonium", "HEP-239-NI", "氮化物", "高浓缩钚-239 氮化物燃料丸", "HEP-239 Nitride Fuel Pellet", 5760, 300, 1.25, 62, 0, 0.075, false},
        {"钚", "Plutonium", "HEP-239-ZA", "锆合金", "高浓缩钚-239-锆合金燃料丸", "HEP-239-Zirconium Alloy Fuel Pellet", 3646, 474, 1.25, 42, 0, 0.075, false},
        {"钚", "Plutonium", "HEP-239-F4", "熔盐 (F4)", "熔融高浓缩钚-239 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HEP-239 Fluoride Fuel", 5760, 300, 1.25, 49, 0, 0.075, false},
        {"钚", "Plutonium", "LEP-241-TRISO", "TRISO", "低浓缩钚-241 TRISO 燃料颗粒", "LEP-241 TRISO Fuel Pebble", 3164, 182, 1.25, 71, 10, 0.075, false},
        {"钚", "Plutonium", "LEP-241-OX", "氧化物", "低浓缩钚-241 氧化物燃料丸", "LEP-241 Oxide Fuel Pellet", 3164, 182, 1.25, 84, 0, 0.075, false},
        {"钚", "Plutonium", "LEP-241-NI", "氮化物", "低浓缩钚-241 氮化物燃料丸", "LEP-241 Nitride Fuel Pellet", 3946, 146, 1.25, 105, 0, 0.075, false},
        {"钚", "Plutonium", "LEP-241-ZA", "锆合金", "低浓缩钚-241-锆合金燃料丸", "LEP-241-Zirconium Alloy Fuel Pellet", 2526, 228, 1.25, 71, 0, 0.075, false},
        {"钚", "Plutonium", "LEP-241-F4", "熔盐 (F4)", "熔融低浓缩钚-241 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LEP-241 Fluoride Fuel", 3946, 146, 1.25, 84, 0, 0.075, false},
        {"钚", "Plutonium", "HEP-241-TRISO", "TRISO", "高浓缩钚-241 TRISO 燃料颗粒", "HEP-241 TRISO Fuel Pebble", 3164, 546, 1.3, 35, 10, 0.075, false},
        {"钚", "Plutonium", "HEP-241-OX", "氧化物", "高浓缩钚-241 氧化物燃料丸", "HEP-241 Oxide Fuel Pellet", 3164, 546, 1.3, 42, 0, 0.075, false},
        {"钚", "Plutonium", "HEP-241-NI", "氮化物", "高浓缩钚-241 氮化物燃料丸", "HEP-241 Nitride Fuel Pellet", 3946, 438, 1.3, 52, 0, 0.075, false},
        {"钚", "Plutonium", "HEP-241-ZA", "锆合金", "高浓缩钚-241-锆合金燃料丸", "HEP-241-Zirconium Alloy Fuel Pellet", 2526, 684, 1.3, 35, 0, 0.075, false},
        {"钚", "Plutonium", "HEP-241-F4", "熔盐 (F4)", "熔融高浓缩钚-241 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HEP-241 Fluoride Fuel", 3946, 438, 1.3, 42, 0, 0.075, false},
        {"混合", "Mixed", "MTRISO-239", "TRISO", "混合 TRISO-239 燃料颗粒", "MTRISO-239 Fuel Pebble", 4354, 132, 1.05, 80, 10, 0.075, false},
        {"混合", "Mixed", "MOX-239", "氧化物", "混合氧化物-239 燃料丸", "MOX-239 Fuel Pellet", 4354, 132, 1.05, 94, 0, 0.075, false},
        {"混合", "Mixed", "MNI-239", "氮化物", "混合氮化物-239 燃料丸", "MNI-239 Fuel Pellet", 5486, 106, 1.05, 118, 0, 0.075, false},
        {"混合", "Mixed", "MZA-239", "锆合金", "混合锆合金-239 燃料丸", "MZA-239 Fuel Pellet", 3472, 166, 1.05, 80, 0, 0.075, false},
        {"混合", "Mixed", "MF4-239", "熔盐 (F4)", "熔融混合燃料氟化物-239", "Molten FLiBe Salt Solution of MF4-239 Fuel", 5486, 106, 1.05, 94, 0, 0.075, false},
        {"混合", "Mixed", "MTRISO-241", "TRISO", "混合 TRISO-241 燃料颗粒", "MTRISO-241 Fuel Pebble", 3014, 192, 1.15, 68, 10, 0.075, false},
        {"混合", "Mixed", "MOX-241", "氧化物", "混合氧化物-241 燃料丸", "MOX-241 Fuel Pellet", 3014, 192, 1.15, 80, 0, 0.075, false},
        {"混合", "Mixed", "MNI-241", "氮化物", "混合氮化物-241 燃料丸", "MNI-241 Fuel Pellet", 3758, 154, 1.15, 100, 0, 0.075, false},
        {"混合", "Mixed", "MZA-241", "锆合金", "混合锆合金-241 燃料丸", "MZA-241 Fuel Pellet", 2406, 240, 1.15, 68, 0, 0.075, false},
        {"混合", "Mixed", "MF4-241", "熔盐 (F4)", "熔融混合燃料氟化物-241", "Molten FLiBe Salt Solution of MF4-241 Fuel", 3758, 154, 1.15, 80, 0, 0.075, false},
        {"镅", "Americium", "LEA-242-TRISO", "TRISO", "低浓缩镅-242 TRISO 燃料颗粒", "LEA-242 TRISO Fuel Pebble", 1476, 390, 1.35, 55, 10, 0.08, false},
        {"镅", "Americium", "LEA-242-OX", "氧化物", "低浓缩镅-242 氧化物燃料丸", "LEA-242 Oxide Fuel Pellet", 1476, 390, 1.35, 65, 0, 0.08, false},
        {"镅", "Americium", "LEA-242-NI", "氮化物", "低浓缩镅-242 氮化物燃料丸", "LEA-242 Nitride Fuel Pellet", 1846, 312, 1.35, 81, 0, 0.08, false},
        {"镅", "Americium", "LEA-242-ZA", "锆合金", "低浓缩镅-242-锆合金燃料丸", "LEA-242-Zirconium Alloy Fuel Pellet", 1180, 488, 1.35, 55, 0, 0.08, false},
        {"镅", "Americium", "LEA-242-F4", "熔盐 (F4)", "熔融低浓缩镅-242 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LEA-242 Fluoride Fuel", 1846, 312, 1.35, 65, 0, 0.08, false},
        {"镅", "Americium", "HEA-242-TRISO", "TRISO", "高浓缩镅-242 TRISO 燃料颗粒", "HEA-242 TRISO Fuel Pebble", 1476, 1170, 1.4, 27, 10, 0.08, false},
        {"镅", "Americium", "HEA-242-OX", "氧化物", "高浓缩镅-242 氧化物燃料丸", "HEA-242 Oxide Fuel Pellet", 1476, 1170, 1.4, 32, 0, 0.08, false},
        {"镅", "Americium", "HEA-242-NI", "氮化物", "高浓缩镅-242 氮化物燃料丸", "HEA-242 Nitride Fuel Pellet", 1846, 936, 1.4, 40, 0, 0.08, false},
        {"镅", "Americium", "HEA-242-ZA", "锆合金", "高浓缩镅-242-锆合金燃料丸", "HEA-242-Zirconium Alloy Fuel Pellet", 1180, 1464, 1.4, 27, 0, 0.08, false},
        {"镅", "Americium", "HEA-242-F4", "熔盐 (F4)", "熔融高浓缩镅-242 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HEA-242 Fluoride Fuel", 1846, 936, 1.4, 32, 0, 0.08, false},
        {"锔", "Curium", "LECm-243-TRISO", "TRISO", "低浓缩锔-243 TRISO 燃料颗粒", "LECm-243 TRISO Fuel Pebble", 1500, 384, 1.45, 56, 10, 0.085, false},
        {"锔", "Curium", "LECm-243-OX", "氧化物", "低浓缩锔-243 氧化物燃料丸", "LECm-243 Oxide Fuel Pellet", 1500, 384, 1.45, 66, 0, 0.085, false},
        {"锔", "Curium", "LECm-243-NI", "氮化物", "低浓缩锔-243 氮化物燃料丸", "LECm-243 Nitride Fuel Pellet", 1870, 308, 1.45, 83, 0, 0.085, false},
        {"锔", "Curium", "LECm-243-ZA", "锆合金", "低浓缩锔-243-锆合金燃料丸", "LECm-243-Zirconium Alloy Fuel Pellet", 1200, 480, 1.45, 56, 0, 0.085, false},
        {"锔", "Curium", "LECm-243-F4", "熔盐 (F4)", "熔融低浓缩锔-243 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LECm-243 Fluoride Fuel", 1870, 308, 1.45, 66, 0, 0.085, false},
        {"锔", "Curium", "HECm-243-TRISO", "TRISO", "高浓缩锔-243 TRISO 燃料颗粒", "HECm-243 TRISO Fuel Pebble", 1500, 1152, 1.5, 28, 10, 0.085, false},
        {"锔", "Curium", "HECm-243-OX", "氧化物", "高浓缩锔-243 氧化物燃料丸", "HECm-243 Oxide Fuel Pellet", 1500, 1152, 1.5, 33, 0, 0.085, false},
        {"锔", "Curium", "HECm-243-NI", "氮化物", "高浓缩锔-243 氮化物燃料丸", "HECm-243 Nitride Fuel Pellet", 1870, 924, 1.5, 41, 0, 0.085, false},
        {"锔", "Curium", "HECm-243-ZA", "锆合金", "高浓缩锔-243-锆合金燃料丸", "HECm-243-Zirconium Alloy Fuel Pellet", 1200, 1440, 1.5, 28, 0, 0.085, false},
        {"锔", "Curium", "HECm-243-F4", "熔盐 (F4)", "熔融高浓缩锔-243 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HECm-243 Fluoride Fuel", 1870, 924, 1.5, 33, 0, 0.085, false},
        {"锔", "Curium", "LECm-245-TRISO", "TRISO", "低浓缩锔-245 TRISO 燃料颗粒", "LECm-245 TRISO Fuel Pebble", 2420, 238, 1.5, 64, 10, 0.085, false},
        {"锔", "Curium", "LECm-245-OX", "氧化物", "低浓缩锔-245 氧化物燃料丸", "LECm-245 Oxide Fuel Pellet", 2420, 238, 1.5, 75, 0, 0.085, false},
        {"锔", "Curium", "LECm-245-NI", "氮化物", "低浓缩锔-245 氮化物燃料丸", "LECm-245 Nitride Fuel Pellet", 3032, 190, 1.5, 94, 0, 0.085, false},
        {"锔", "Curium", "LECm-245-ZA", "锆合金", "低浓缩锔-245-锆合金燃料丸", "LECm-245-Zirconium Alloy Fuel Pellet", 1932, 298, 1.5, 64, 0, 0.085, false},
        {"锔", "Curium", "LECm-245-F4", "熔盐 (F4)", "熔融低浓缩锔-245 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LECm-245 Fluoride Fuel", 3032, 190, 1.5, 75, 0, 0.085, false},
        {"锔", "Curium", "HECm-245-TRISO", "TRISO", "高浓缩锔-245 TRISO 燃料颗粒", "HECm-245 TRISO Fuel Pebble", 2420, 714, 1.55, 32, 10, 0.085, false},
        {"锔", "Curium", "HECm-245-OX", "氧化物", "高浓缩锔-245 氧化物燃料丸", "HECm-245 Oxide Fuel Pellet", 2420, 714, 1.55, 37, 0, 0.085, false},
        {"锔", "Curium", "HECm-245-NI", "氮化物", "高浓缩锔-245 氮化物燃料丸", "HECm-245 Nitride Fuel Pellet", 3032, 570, 1.55, 47, 0, 0.085, false},
        {"锔", "Curium", "HECm-245-ZA", "锆合金", "高浓缩锔-245-锆合金燃料丸", "HECm-245-Zirconium Alloy Fuel Pellet", 1932, 894, 1.55, 32, 0, 0.085, false},
        {"锔", "Curium", "HECm-245-F4", "熔盐 (F4)", "熔融高浓缩锔-245 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HECm-245 Fluoride Fuel", 3032, 570, 1.55, 37, 0, 0.085, false},
        {"锔", "Curium", "LECm-247-TRISO", "TRISO", "低浓缩锔-247 TRISO 燃料颗粒", "LECm-247 TRISO Fuel Pebble", 2150, 268, 1.55, 61, 10, 0.085, false},
        {"锔", "Curium", "LECm-247-OX", "氧化物", "低浓缩锔-247 氧化物燃料丸", "LECm-247 Oxide Fuel Pellet", 2150, 268, 1.55, 72, 0, 0.085, false},
        {"锔", "Curium", "LECm-247-NI", "氮化物", "低浓缩锔-247 氮化物燃料丸", "LECm-247 Nitride Fuel Pellet", 2692, 214, 1.55, 90, 0, 0.085, false},
        {"锔", "Curium", "LECm-247-ZA", "锆合金", "低浓缩锔-247-锆合金燃料丸", "LECm-247-Zirconium Alloy Fuel Pellet", 1714, 336, 1.55, 61, 0, 0.085, false},
        {"锔", "Curium", "LECm-247-F4", "熔盐 (F4)", "熔融低浓缩锔-247 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LECm-247 Fluoride Fuel", 2692, 214, 1.55, 72, 0, 0.085, false},
        {"锔", "Curium", "HECm-247-TRISO", "TRISO", "高浓缩锔-247 TRISO 燃料颗粒", "HECm-247 TRISO Fuel Pebble", 2150, 804, 1.6, 30, 10, 0.085, false},
        {"锔", "Curium", "HECm-247-OX", "氧化物", "高浓缩锔-247 氧化物燃料丸", "HECm-247 Oxide Fuel Pellet", 2150, 804, 1.6, 36, 0, 0.085, false},
        {"锔", "Curium", "HECm-247-NI", "氮化物", "高浓缩锔-247 氮化物燃料丸", "HECm-247 Nitride Fuel Pellet", 2692, 642, 1.6, 45, 0, 0.085, false},
        {"锔", "Curium", "HECm-247-ZA", "锆合金", "高浓缩锔-247-锆合金燃料丸", "HECm-247-Zirconium Alloy Fuel Pellet", 1714, 1008, 1.6, 30, 0, 0.085, false},
        {"锔", "Curium", "HECm-247-F4", "熔盐 (F4)", "熔融高浓缩锔-247 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HECm-247 Fluoride Fuel", 2692, 642, 1.6, 36, 0, 0.085, false},
        {"锫", "Berkelium", "LEB-248-TRISO", "TRISO", "低浓缩锫-248 TRISO 燃料颗粒", "LEB-248 TRISO Fuel Pebble", 2166, 266, 1.65, 62, 10, 0.09, false},
        {"锫", "Berkelium", "LEB-248-OX", "氧化物", "低浓缩锫-248 氧化物燃料丸", "LEB-248 Oxide Fuel Pellet", 2166, 266, 1.65, 73, 0, 0.09, false},
        {"锫", "Berkelium", "LEB-248-NI", "氮化物", "低浓缩锫-248 氮化物燃料丸", "LEB-248 Nitride Fuel Pellet", 2716, 212, 1.65, 91, 0, 0.09, false},
        {"锫", "Berkelium", "LEB-248-ZA", "锆合金", "低浓缩锫-248-锆合金燃料丸", "LEB-248-Zirconium Alloy Fuel Pellet", 1734, 332, 1.65, 62, 0, 0.09, false},
        {"锫", "Berkelium", "LEB-248-F4", "熔盐 (F4)", "熔融低浓缩锫-248 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LEB-248 Fluoride Fuel", 2716, 212, 1.65, 73, 0, 0.09, false},
        {"锫", "Berkelium", "HEB-248-TRISO", "TRISO", "高浓缩锫-248 TRISO 燃料颗粒", "HEB-248 TRISO Fuel Pebble", 2166, 798, 1.7, 31, 10, 0.09, false},
        {"锫", "Berkelium", "HEB-248-OX", "氧化物", "高浓缩锫-248 氧化物燃料丸", "HEB-248 Oxide Fuel Pellet", 2166, 798, 1.7, 36, 0, 0.09, false},
        {"锫", "Berkelium", "HEB-248-NI", "氮化物", "高浓缩锫-248 氮化物燃料丸", "HEB-248 Nitride Fuel Pellet", 2716, 636, 1.7, 45, 0, 0.09, false},
        {"锫", "Berkelium", "HEB-248-ZA", "锆合金", "高浓缩锫-248-锆合金燃料丸", "HEB-248-Zirconium Alloy Fuel Pellet", 1734, 996, 1.7, 31, 0, 0.09, false},
        {"锫", "Berkelium", "HEB-248-F4", "熔盐 (F4)", "熔融高浓缩锫-248 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HEB-248 Fluoride Fuel", 2716, 636, 1.7, 36, 0, 0.09, false},
        {"锎", "Californium", "LECf-249-TRISO", "TRISO", "低浓缩锎-249 TRISO 燃料颗粒", "LECf-249 TRISO Fuel Pebble", 1066, 540, 1.75, 51, 10, 0.1, true},
        {"锎", "Californium", "LECf-249-OX", "氧化物", "低浓缩锎-249 氧化物燃料丸", "LECf-249 Oxide Fuel Pellet", 1066, 540, 1.75, 60, 0, 0.1, true},
        {"锎", "Californium", "LECf-249-NI", "氮化物", "低浓缩锎-249 氮化物燃料丸", "LECf-249 Nitride Fuel Pellet", 1334, 432, 1.75, 75, 0, 0.1, true},
        {"锎", "Californium", "LECf-249-ZA", "锆合金", "低浓缩锎-249-锆合金燃料丸", "LECf-249-Zirconium Alloy Fuel Pellet", 852, 676, 1.75, 51, 0, 0.1, true},
        {"锎", "Californium", "LECf-249-F4", "熔盐 (F4)", "熔融低浓缩锎-249 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LECf-249 Fluoride Fuel", 1334, 432, 1.75, 60, 0, 0.1, true},
        {"锎", "Californium", "HECf-249-TRISO", "TRISO", "高浓缩锎-249 TRISO 燃料颗粒", "HECf-249 TRISO Fuel Pebble", 1066, 1620, 1.8, 25, 10, 0.1, true},
        {"锎", "Californium", "HECf-249-OX", "氧化物", "高浓缩锎-249 氧化物燃料丸", "HECf-249 Oxide Fuel Pellet", 1066, 1620, 1.8, 30, 0, 0.1, true},
        {"锎", "Californium", "HECf-249-NI", "氮化物", "高浓缩锎-249 氮化物燃料丸", "HECf-249 Nitride Fuel Pellet", 1334, 1296, 1.8, 37, 0, 0.1, true},
        {"锎", "Californium", "HECf-249-ZA", "锆合金", "高浓缩锎-249-锆合金燃料丸", "HECf-249-Zirconium Alloy Fuel Pellet", 852, 2028, 1.8, 25, 0, 0.1, true},
        {"锎", "Californium", "HECf-249-F4", "熔盐 (F4)", "熔融高浓缩锎-249 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HECf-249 Fluoride Fuel", 1334, 1296, 1.8, 30, 0, 0.1, true},
        {"锎", "Californium", "LECf-251-TRISO", "TRISO", "低浓缩锎-251 TRISO 燃料颗粒", "LECf-251 TRISO Fuel Pebble", 2000, 288, 1.8, 60, 10, 0.1, true},
        {"锎", "Californium", "LECf-251-OX", "氧化物", "低浓缩锎-251 氧化物燃料丸", "LECf-251 Oxide Fuel Pellet", 2000, 288, 1.8, 71, 0, 0.1, true},
        {"锎", "Californium", "LECf-251-NI", "氮化物", "低浓缩锎-251 氮化物燃料丸", "LECf-251 Nitride Fuel Pellet", 2504, 230, 1.8, 89, 0, 0.1, true},
        {"锎", "Californium", "LECf-251-ZA", "锆合金", "低浓缩锎-251-锆合金燃料丸", "LECf-251-Zirconium Alloy Fuel Pellet", 1600, 360, 1.8, 60, 0, 0.1, true},
        {"锎", "Californium", "LECf-251-F4", "熔盐 (F4)", "熔融低浓缩锎-251 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of LECf-251 Fluoride Fuel", 2504, 230, 1.8, 71, 0, 0.1, true},
        {"锎", "Californium", "HECf-251-TRISO", "TRISO", "高浓缩锎-251 TRISO 燃料颗粒", "HECf-251 TRISO Fuel Pebble", 2000, 864, 1.85, 30, 10, 0.1, true},
        {"锎", "Californium", "HECf-251-OX", "氧化物", "高浓缩锎-251 氧化物燃料丸", "HECf-251 Oxide Fuel Pellet", 2000, 864, 1.85, 35, 0, 0.1, true},
        {"锎", "Californium", "HECf-251-NI", "氮化物", "高浓缩锎-251 氮化物燃料丸", "HECf-251 Nitride Fuel Pellet", 2504, 690, 1.85, 44, 0, 0.1, true},
        {"锎", "Californium", "HECf-251-ZA", "锆合金", "高浓缩锎-251-锆合金燃料丸", "HECf-251-Zirconium Alloy Fuel Pellet", 1600, 1080, 1.85, 30, 0, 0.1, true},
        {"锎", "Californium", "HECf-251-F4", "熔盐 (F4)", "熔融高浓缩锎-251 氟化物燃料氟锂铍盐溶液", "Molten FLiBe Salt Solution of HECf-251 Fluoride Fuel", 2504, 690, 1.85, 35, 0, 0.1, true},
    };
    return data;
}

const std::vector<SinkType>& sinkTypes() {
    static const std::vector<SinkType> data = {
        {0, "water_sink", "water", "水裂变散热器", "Water Fission Heat Sink", 55, "one cell"},
        {1, "iron_sink", "iron", "铁裂变散热器", "Iron Fission Heat Sink", 50, "one moderator"},
        {2, "redstone_sink", "redstone", "红石裂变散热器", "Redstone Fission Heat Sink", 85, "one cell && one moderator"},
        {3, "quartz_sink", "quartz", "石英裂变散热器", "Quartz Fission Heat Sink", 80, "one redstone sink"},
        {4, "obsidian_sink", "obsidian", "黑曜石裂变散热器", "Obsidian Fission Heat Sink", 70, "two axial glowstone sinks"},
        {5, "nether_brick_sink", "nether_brick", "地狱砖裂变散热器", "Nether Brick Fission Heat Sink", 105, "one obsidian sink"},
        {6, "glowstone_sink", "glowstone", "荧石裂变散热器", "Glowstone Fission Heat Sink", 90, "two moderators"},
        {7, "lapis_sink", "lapis", "青金石裂变散热器", "Lapis Fission Heat Sink", 100, "one cell && one casing"},
        {8, "gold_sink", "gold", "金裂变散热器", "Gold Fission Heat Sink", 110, "exactly two iron sinks"},
        {9, "prismarine_sink", "prismarine", "海晶石裂变散热器", "Prismarine Fission Heat Sink", 115, "two water sinks"},
        {10, "slime_sink", "slime", "史莱姆裂变散热器", "Slime Fission Heat Sink", 145, "exactly one water sink && two lead sinks"},
        {11, "end_stone_sink", "end_stone", "末地石裂变散热器", "End Stone Fission Heat Sink", 65, "one reflector"},
        {12, "purpur_sink", "purpur", "紫珀裂变散热器", "Purpur Fission Heat Sink", 95, "one reflector && one iron sink"},
        {13, "diamond_sink", "diamond", "钻石裂变散热器", "Diamond Fission Heat Sink", 200, "one cell && one gold sink"},
        {14, "emerald_sink", "emerald", "绿宝石裂变散热器", "Emerald Fission Heat Sink", 195, "one moderator && one prismarine sink"},
        {15, "copper_sink", "copper", "铜裂变散热器", "Copper Fission Heat Sink", 75, "one water sink"},
        {16, "tin_sink", "tin", "锡裂变散热器", "Tin Fission Heat Sink", 120, "two axial lapis sinks"},
        {17, "lead_sink", "lead", "铅裂变散热器", "Lead Fission Heat Sink", 60, "one iron sink"},
        {18, "boron_sink", "boron", "硼裂变散热器", "Boron Fission Heat Sink", 160, "exactly one quartz sink && one casing"},
        {19, "lithium_sink", "lithium", "锂裂变散热器", "Lithium Fission Heat Sink", 130, "exactly two axial lead sinks && one casing"},
        {20, "magnesium_sink", "magnesium", "镁裂变散热器", "Magnesium Fission Heat Sink", 125, "exactly one moderator && one casing"},
        {21, "manganese_sink", "manganese", "锰裂变散热器", "Manganese Fission Heat Sink", 150, "two cells"},
        {22, "aluminum_sink", "aluminum", "铝裂变散热器", "Aluminum Fission Heat Sink", 175, "one quartz sink && one lapis sink"},
        {23, "silver_sink", "silver", "银裂变散热器", "Silver Fission Heat Sink", 170, "two glowstone sinks && one tin sink"},
        {24, "fluorite_sink", "fluorite", "氟石裂变散热器", "Fluorite Fission Heat Sink", 165, "one gold sink && one prismarine sink"},
        {25, "villiaumite_sink", "villiaumite", "氟盐裂变散热器", "Villiaumite Fission Heat Sink", 180, "one redstone sink && one end_stone sink"},
        {26, "carobbiite_sink", "carobbiite", "方氟钾石裂变散热器", "Carobbiite Fission Heat Sink", 140, "one end_stone sink && one copper sink"},
        {27, "arsenic_sink", "arsenic", "砷裂变散热器", "Arsenic Fission Heat Sink", 135, "two axial reflectors"},
        {28, "liquid_nitrogen_sink", "liquid_nitrogen", "液氮裂变散热器", "Liquid Nitrogen Fission Heat Sink", 185, "two copper sinks && one purpur sink"},
        {29, "liquid_helium_sink", "liquid_helium", "液氦裂变散热器", "Liquid Helium Fission Heat Sink", 190, "exactly two redstone sinks"},
        {30, "enderium_sink", "enderium", "末影裂变散热器", "Enderium Fission Heat Sink", 155, "three moderators"},
        {31, "cryotheum_sink", "cryotheum", "凛冰裂变散热器", "Cryotheum Fission Heat Sink", 205, "three cells"},
        {32, "sodium_sink", "sodium", "钠裂变散热器", "Sodium Fission Heat Sink", 160, "two axial lithium sinks"},
        {33, "potassium_sink", "potassium", "钾裂变散热器", "Potassium Fission Heat Sink", 200, "exactly one sodium sink"},
        {34, "rubidium_sink", "rubidium", "铷裂变散热器", "Rubidium Fission Heat Sink", 240, "exactly one potassium sink"},
        {35, "cesium_sink", "cesium", "铯裂变散热器", "Cesium Fission Heat Sink", 280, "exactly one rubidium sink"},
        {36, "pyro_sink", "pyro", "烈焰之炽焱裂变散热器", "Pyrotheum Heat Sink", 290, "five cells"},
        {37, "aero_sink", "aero", "飞扬之清风裂变散热器", "Aerotheum Heat Sink", 90, "one liquid_nitrogen sink || one liquid_helium sink"},
        {38, "petro_sink", "petro", "板块之层岩裂变散热器", "Petrotheum Heat Sink", 170, "four moderators"},
        {39, "nickel_sink", "nickel", "镍裂变散热器", "Nickel Heat Sink", 120, "two axial obsidian sinks"},
        {40, "platinum_sink", "platinum", "铂裂变散热器", "Platinum Heat Sink", 145, "four iron sinks && exactly zero casing"},
        {41, "iridium_sink", "iridium", "铱裂变散热器", "Iridium Heat Sink", 340, "six cells"},
        {42, "signalum_sink", "signalum", "信素裂变散热器", "Signalum Heat Sink", 250, "one redstone sink && one ferroboron sink"},
        {43, "lumium_sink", "lumium", "流明裂变散热器", "Lumium Heat Sink", 175, "four glowstone sinks"},
        {44, "mana_dust_sink", "mana_dust", "元始魔力粉裂变散热器", "Mana Dust Heat Sink", 80, "exactly three casing"},
        {45, "mithril_sink", "mithril", "秘银裂变散热器", "Mithril Heat Sink", 120, "two axial enderium sinks && exactly zero moderator"},
        {46, "invar_sink", "invar", "殷钢裂变散热器", "Invar Heat Sink", 105, "one lumium sink && one water sink"},
        {47, "constantan_sink", "constantan", "康铜裂变散热器", "Constantan Heat Sink", 120, "one copper sink && one nickel sink"},
        {48, "bronze_sink", "bronze", "青铜裂变散热器", "Bronze Heat Sink", 90, "three copper sinks && one tin sink"},
        {49, "electrum_sink", "electrum", "琥珀金裂变散热器", "Electrum Heat Sink", 195, "two axial silver sinks"},
        {50, "steel_sink", "steel", "钢裂变散热器", "Steel Fission Heat Sink", 125, "three iron sinks"},
        {51, "ferroboron_sink", "ferroboron", "硼铁合金裂变散热器", "Ferroboron Fission Heat Sink", 225, "one gold sink && two axial cells"},
        {52, "tough_alloy_sink", "tough_alloy", "高强合金裂变散热器", "Tough Alloy Fission Heat Sink", 120, "one villiaumite sink"},
        {53, "limno2_sink", "limno2", "锰酸锂裂变散热器", "Lithium Manganese Dioxide Fission Heat Sink", 190, "two axial lithium sinks && one manganese sink"},
        {54, "mgb2_sink", "mgb2", "二硼化镁裂变散热器", "Magnesium Diboride Fission Heat Sink", 220, "one magnesium sink && one boron sink"},
        {55, "boron_arsenide_sink", "boron_arsenide", "砷化硼裂变散热器", "Boron Arsenide Fission Heat Sink", 70, "one steel sink && three lead sinks"},
        {56, "boron_nitride_sink", "boron_nitride", "氮化硼裂变散热器", "Boron Nitride Fission Heat Sink", 160, "one end_stone sink && one liquid_nitrogen sink"},
        {57, "rhodochrosite_sink", "rhodochrosite", "菱锰裂变散热器", "Rhodochrosite Fission Heat Sink", 175, "one emerald sink && one redstone sink && exactly zero casing"},
        {58, "zirconium_sink", "zirconium", "锆裂变散热器", "Zirconium Fission Heat Sink", 145, "two axial gold sinks"},
        {59, "hard_carbon_sink", "hard_carbon", "硬碳裂变散热器", "Hard Carbon Fission Heat Sink", 145, "one diamond sink && one moderator"},
        {60, "extreme_alloy_sink", "extreme_alloy", "极限合金裂变散热器", "Extreme Alloy Fission Heat Sink", 180, "one tough_alloy sink && one hard_carbon sink"},
        {61, "thermoconducting_alloy_sink", "thermoconducting_alloy", "导热合金裂变散热器", "Thermoconducting Alloy Fission Heat Sink", 240, "two axial extreme_alloy sinks"},
        {62, "zircaloy_sink", "zircaloy", "锆锡合金裂变散热器", "Zircaloy Fission Heat Sink", 175, "one zirconium sink && one casing"},
        {63, "sic_sic_cmc_sink", "sic_sic_cmc", "碳化硅陶瓷基复合材料裂变散热器", "SiC-SiC CMC Fission Heat Sink", 110, "two axial quartz sinks"},
        {64, "hsla_sink", "hsla", "高强度低合金钢裂变散热器", "HSLA Steel Fission Heat Sink", 215, "two reflectors && one diamond sink"},
        {65, "smore_sink", "smore", "棉花糖巧克力夹心裂变散热器", "S'more Fission Heat Sink", 563, "two axial liquid_helium sinks && two axial liquid_nitrogen sinks && two axial thermoconducting_alloy sinks"},
        {66, "liquid_oxygen_sink", "liquid_oxygen", "液氧裂变散热器", "Liquid Oxygen Heat Sink", 30, "one lead sink"},
        {67, "corium_sink", "corium", "堆芯熔融物裂变散热器", "Corium Heat Sink", -5, "exactly zero casing"},
        {68, "sea_lantern_sink", "sea_lantern", "海晶灯裂变散热器", "Sea Lantern Heat Sink", 80, "three prismarine sinks"},
        {69, "tnt_sink", "tnt", "TNT 裂变散热器", "TNT Heat Sink", 40, "one shield"},
        {70, "bone_sink", "bone", "骨头裂变散热器", "Bone Heat Sink", 40, "one irradiator"},
        {71, "milk_sink", "milk", "牛奶裂变散热器", "Milk Heat Sink", 60, "one bone sink"},
        {72, "brick_sink", "brick", "砖块裂变散热器", "Brick Heat Sink", 65, "one milk sink && one lead sink"},
        {73, "lava_sink", "lava", "熔岩裂变散热器", "Lava Heat Sink", 75, "two axial irradiators"},
        {74, "magma_slime_sink", "magma_slime", "岩浆膏裂变散热器", "Magma Cream Heat Sink", 40, "one lead sink && one gold sink"},
        {75, "sponge_sink", "sponge", "海绵裂变散热器", "Sponge Heat Sink", 70, "one irradiator && one shield"},
        {76, "blue_ice_sink", "blue_ice", "浮冰裂变散热器", "Packed Ice Heat Sink", 55, "two shields"},
        {77, "bismuth_sink", "bismuth", "铋裂变散热器", "Bismuth Heat Sink", 145, "one shield && exactly two iron sinks"},
        {78, "glowshroom_sink", "glowshroom", "荧核菇裂变散热器", "Glowshroom Heat Sink", 95, "two lava sinks"},
        {79, "silicon_sink", "silicon", "硅裂变散热器", "Silicon Heat Sink", 70, "one shield && one moderator"},
        {80, "sic_sink", "sic", "碳化硅裂变散热器", "Silicon Carbide Heat Sink", 135, "one silicon sink && one end_stone sink"},
        {81, "certus_sink", "certus", "赛特斯石英裂变散热器", "Certus Quartz Heat Sink", 80, "one quartz sink"},
        {82, "fluix_sink", "fluix", "福鲁伊克斯裂变散热器", "Fluix Crystal Heat Sink", 115, "one certus sink && one water sink"},
        {83, "fluix_steel_sink", "fluix_steel", "福鲁伊克斯钢裂变散热器", "Fluix Steel Heat Sink", 150, "one fluix sink && one steel sink"},
        {84, "fluxed_electrum_sink", "fluxed_electrum", "红石琥珀金裂变散热器", "Fluxed Electrum Heat Sink", 160, "one redstone sink && one electrum sink"},
        {85, "flux_crystal_sink", "flux_crystal", "红石水晶裂变散热器", "Fluxed Crystal Heat Sink", 135, "one redstone sink && one gold sink"},
    };
    return data;
}

const std::vector<ModeratorType>& moderatorTypes() {
    static const std::vector<ModeratorType> data = {
        {"blockGraphite", "石墨块", "Graphite Block", 10, 1.1},
        {"blockBeryllium", "铍块", "Beryllium Block", 22, 1.05},
        {"nuclearcraft:heavy_water_moderator", "重水减速剂", "Heavy Water Moderator", 36, 1},
        {"contenttweaker:water_mod", "轻水减速剂", "Light Water Moderator", 29, 0.95},
        {"contenttweaker:pu_schmeared_be_mod", "钚-238 涂层铍减速剂", "Plutonium-238 Schmeared Beryllium Moderator", 49, 0.98},
        {"contenttweaker:hydrocarbon_mod", "烃基减速剂", "Hydrocarbon Moderator", 3, 1.02},
    };
    return data;
}

const std::vector<ReflectorType>& reflectorTypes() {
    static const std::vector<ReflectorType> data = {
        {"nuclearcraft:fission_reflector:0", "铍-碳中子反射器", "Beryllium-Carbon Neutron Reflector", 0.5, 1},
        {"nuclearcraft:fission_reflector:1", "铅-钢中子反射器", "Lead-Steel Neutron Reflector", 0.25, 0.5},
        {"contenttweaker:cf_neutron_multiplier", "锎中子反射器（增殖器）", "Neutron Reflector-Multiplier", 0.65, 1.65},
        {"contenttweaker:gold_reflector", "金-碳中子反射器", "Gold-Carbon Neutron Reflector", 0.6, 0.8},
    };
    return data;
}

const std::vector<SourceType>& sourceTypes() {
    static const std::vector<SourceType> data = {
        {"nuclearcraft:fission_source:0", "镭-铍裂变中子源", "Ra-Be Fission Neutron Source", 0.9},
        {"nuclearcraft:fission_source:1", "钋-铍裂变中子源", "Po-Be Fission Neutron Source", 0.95},
        {"nuclearcraft:fission_source:2", "锎-252 裂变中子源", "Cf-252 Fission Neutron Source", 1},
    };
    return data;
}

const std::vector<ShieldType>& shieldTypes() {
    static const std::vector<ShieldType> data = {
        {"nuclearcraft:fission_shield:0", "硼-银中子防护屏", "Boron-Silver Neutron Shield", 5, 0.5},
        {"nuclearcraft:fission_shield:1", "钆-钐中子防护屏", "Gadolinium-Samarium Neutron Shield", 10, 0.75},
    };
    return data;
}

const std::vector<IrradiatorRecipeType>& irradiatorRecipeTypes() {
    static const std::vector<IrradiatorRecipeType> data = {
        {"ingotThorium/dustThorium", "dustTBP", "钍辐照为钍增殖镤粉", "Thorium to TBP Irradiator Recipe", 0, 0},
        {"ingotTBP/dustTBP", "dustProtactinium233", "钍增殖镤辐照为镤-233粉", "TBP to Protactinium-233 Irradiator Recipe", 0, 0},
        {"ingotBismuth/dustBismuth", "dustPolonium", "铋辐照为钋粉", "Bismuth to Polonium Irradiator Recipe", 0, 0.5},
    };
    return data;
}

int defaultIrradiatorRecipeIndex() {
    const auto& recipes = irradiatorRecipeTypes();
    int best = 0;
    for (int i = 1; i < static_cast<int>(recipes.size()); ++i) {
        const auto& candidate = recipes.at(static_cast<size_t>(i));
        const auto& current = recipes.at(static_cast<size_t>(best));
        if (candidate.heatPerFlux < current.heatPerFlux ||
            (candidate.heatPerFlux == current.heatPerFlux && candidate.efficiency > current.efficiency)) {
            best = i;
        }
    }
    return best;
}

} // namespace ncfr
