cmake_minimum_required(VERSION 3.20)

project(FissionReactorOptimizer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 6.9.1 EXACT REQUIRED COMPONENTS Widgets)

if (MSVC)
    add_compile_options(/utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

qt_standard_project_setup()

qt_add_executable(fission_reactor
    WIN32
    src/main.cpp
    src/Data.cpp
    src/FuelPlacementPrefilter.cpp
    src/Grid.cpp
    src/MainWindow.cpp
    src/NeutronLineTraversal.cpp
    src/Optimizer.cpp
    src/OptimizerCommon.cpp
    src/OptimizerConductorBridge.cpp
    src/OptimizerCooling.cpp
    src/OptimizerDirectional.cpp
    src/OptimizerDirectionalCompaction.cpp
    src/OptimizerIrradiator.cpp
    src/OptimizerMerge.cpp
    src/OptimizerMixedSpecialCooling.cpp
    src/OptimizerSingle.cpp
    src/OptimizerSingleSpecialCooling.cpp
    src/OptimizerSpecialCoolingCommon.cpp
    src/Perf.cpp
    src/ReactorAssembly.cpp
    src/ReactorGridWidget.cpp
    src/ReactorMetrics.cpp
    src/Rule.cpp
    src/Simulator.cpp
    include/Data.h
    include/FuelPlacementPrefilter.h
    include/FuelSpecialCases.h
    include/Grid.h
    include/MainWindow.h
    include/NeutronLineTraversal.h
    include/NeutronRules.h
    include/Optimizer.h
    include/OptimizerCommon.h
    include/OptimizerConductorBridge.h
    include/OptimizerCooling.h
    include/OptimizerDiagnostics.h
    include/OptimizerDirectional.h
    include/OptimizerIrradiator.h
    include/OptimizerMerge.h
    include/OptimizerSingle.h
    include/OptimizerSpecialCooling.h
    include/OptimizerTypes.h
    include/Output.h
    include/Perf.h
    include/ReactorAssembly.h
    include/ReactorGridWidget.h
    include/ReactorMetrics.h
    include/Rule.h
    include/Simulator.h
    include/StateVector.h
)

target_include_directories(fission_reactor PRIVATE include)
target_link_libraries(fission_reactor PRIVATE Qt6::Widgets)
