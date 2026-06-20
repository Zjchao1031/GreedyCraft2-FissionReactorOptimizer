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
    src/FuelPlacementPrefilter.h
    src/Grid.cpp
    src/MainWindow.cpp
    src/MainWindow.h
    src/Optimizer.cpp
    src/OptimizerCommon.cpp
    src/OptimizerCooling.cpp
    src/OptimizerDetail.h
    src/OptimizerIrradiator.cpp
    src/OptimizerMerge.cpp
    src/OptimizerSingle.cpp
    src/Perf.cpp
    src/Perf.h
    src/ReactorGridWidget.cpp
    src/ReactorGridWidget.h
    src/Rule.cpp
    src/Simulator.cpp
    src/StateVector.h
)

target_include_directories(fission_reactor PRIVATE src)
target_link_libraries(fission_reactor PRIVATE Qt6::Widgets)
