# GreedyCraft2-FissionReactorOptimizer

GreedyCraft2-FissionReactorOptimizer 是一个面向 GreedyCraft 2 中 NuclearCraft 固体裂变反应堆的布局生成工具。它读取内置的燃料、散热器、减速剂、反射器、中子源、防护屏和辐照器配方数据，根据选择的燃料组合自动搜索可运行、可冷却、功能块连通的反应堆方案，并用 Qt 图形界面按层展示结果。

这个项目主要解决的问题是：在 GreedyCraft2 的 NuclearCraft 配置和 CraftTweaker 动态散热器规则下，快速生成一个可以直接参考搭建的固体裂变反应堆结构。

## 功能

- 支持普通结构生成：1、2、4 个燃料单元。
- 支持辐照结构生成：固定 6 个燃料单元 + 1 个中心辐照仓。
- 自动判断非自启动燃料需要的外壁中子源数量和方向。
- 模拟燃料临界、通量、原始发热、散热量、散热器摆放规则、功能块连通性和冷却余量。
- 按二维分层视图展示三维反应堆结构，并提供方块类型图例。
- 支持在结果中选择燃料单元，查看可替换燃料候选。
- 支持将生成结果导出为 JSON，包含输入燃料、内部/外部尺寸、指标和逐层网格。

优化器使用启发式搜索和局部改良策略，目标是生成实用可运行方案，而不是证明全局最优。

## 数据来源

仓库中的 `NuclearCraft_FissionReactor_SourceData.xlsx` 是项目的数据审计表，源码中的 `src/Data.cpp` 由这份表格生成。工作簿包含 9 个工作表：

- `说明与来源`：数据口径、生成时间、来源文件说明。
- `固体裂变燃料`：TRISO、Oxide、Nitride、Zirconium Alloy、F4 五类燃料形态及其时间、发热、效率、临界、通量等参数。
- `散热器`：标准和动态注册散热器的冷却率与摆放规则。
- `减速剂`：通量因子和效率倍率。
- `反射器`：效率倍率和反射率。
- `中子源`：中子源效率。
- `中子防护屏`：每通量产热和效率倍率。
- `辐照器配方`：输入、输出、每通量产热和效率倍率。
- `配置映射`：NuclearCraft 配置项与表格字段的对应关系。

表格数据来自 NuclearCraft 源码、GreedyCraft2 的 `nuclearcraft.cfg`、本地化文件，以及 GreedyCraft2 中用于注册动态散热器的 CraftTweaker 脚本。当前口径不收录 IC2 Uranium、IC2 MOX、Yellorium/Blutonium 等兼容燃料。

## 构建环境

项目使用 C++17、CMake 和 Qt Widgets：

- CMake 3.20 或更高版本
- 支持 C++17 的编译器
- Qt 6.9.1，且需要 `Widgets` 组件

`CMakeLists.txt` 当前使用：

```cmake
find_package(Qt6 6.9.1 EXACT REQUIRED COMPONENTS Widgets)
```

如果你的 Qt 版本不是 6.9.1，需要先修改 `CMakeLists.txt` 中的版本要求。

## 构建

在仓库根目录执行：

```powershell
cmake -S . -B build -DCMAKE_PREFIX_PATH="C:\Qt\6.9.1\msvc2022_64"
cmake --build build --config Release
```

如果使用 MinGW，请把 `CMAKE_PREFIX_PATH` 改成对应的 Qt MinGW 安装目录，例如：

```powershell
cmake -S . -B build-mingw -G "MinGW Makefiles" -DCMAKE_PREFIX_PATH="C:\Qt\6.9.1\mingw_64"
cmake --build build-mingw
```

构建目标名为 `fission_reactor`。

## 使用

启动程序后，主界面标题为“裂变反应堆搭建优化器”。

普通结构生成：

1. 在“普通结构生成”区域选择燃料单元数量，可选 1、2、4。
2. 为每个燃料单元选择燃料族和具体燃料。
3. 点击“生成方案”。
4. 在右侧“二维分层方案”中切换层数查看结构。

辐照结构生成：

1. 在“辐照结构生成”区域选择 5 个燃料。
2. 点击“生成方案”。
3. 程序会尝试生成中心辐照仓结构，使 5 个燃料围绕中心辐照仓运行。

结果生成后，可以点击“将结果导出为JSON格式”。导出的文件会保存在项目目录下，文件名形如 `Result 0.json`、`Result 1.json`。

## JSON 输出

导出 JSON 的 schema 为：

```json
{
  "schema": "nuclearcraft-fission-reactor-result",
  "schemaVersion": 4,
  "request": {},
  "internalSize": {},
  "externalSize": {},
  "metrics": {},
  "grid": {}
}
```

其中 `metrics` 包含：

- `minCoolingMargin`：最小冷却余量，单位 H/t。
- `usefulBlocks`：有效功能块数量。
- `disconnectedFunctionalBlocks`：未连通的功能块数量。
- `functionalIrradiators`：有效辐照仓数量。
- `irradiatorFlux`：辐照仓总通量。

`grid.layers` 使用外部网格零基坐标；边界坐标为外壳，内部坐标范围为 `1..内部尺寸`。

## 源码结构

- `src/main.cpp`：Qt 应用入口。
- `src/MainWindow.*`：主窗口、输入面板、结果展示、燃料替换和 JSON 导出。
- `src/ReactorGridWidget.*`：二维分层网格绘制。
- `src/Data.*`：嵌入式 NuclearCraft/GreedyCraft2 数据。
- `src/Grid.*`：反应堆三维网格、方块类型和坐标工具。
- `src/Rule.*`：散热器摆放规则解析和校验。
- `src/Simulator.*`：通量、发热、冷却、功能块和集群连通性模拟。
- `src/FuelPlacementPrefilter.*`：燃料关系预筛选。
- `src/Optimizer*`：普通结构、合并结构、散热扩展和中心辐照仓结构优化。
- `src/Output.*`：文本输出辅助。
- `src/Perf.*`：Debug 构建下的性能统计和日志。

## 当前限制

- 目前只实现 1、2、4 个普通燃料单元，以及 6 燃料中心辐照仓模式。
- 优化过程是启发式的，结果不保证体积最小、材料最少或收益全局最优。
- 数据口径绑定 GreedyCraft2 当前表格来源；如果整合包配置或脚本发生变化，需要同步更新数据表和 `src/Data.cpp`。
- 程序显示的“任意中子源”表示需要一个能指向对应燃料的外壁中子源，实际搭建时请使用可用的 NuclearCraft 中子源方块。

## License

本项目使用 MIT License。你可以自由使用、修改、发布和分发本项目代码，但需要保留原始版权声明和许可证文本。
