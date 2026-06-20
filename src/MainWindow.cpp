#include "MainWindow.h"

#include "Data.h"
#include "ReactorGridWidget.h"
#include "Simulator.h"

#include <QApplication>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QList>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMessageBox>
#include <QMetaObject>
#include <QPair>
#include <QProgressBar>
#include <QPushButton>
#include <QStringList>
#include <QSpinBox>
#include <QStatusBar>
#include <QThread>
#include <QTimer>
#include <QVariant>
#include <QVBoxLayout>

#include <algorithm>
#include <exception>
#include <functional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

QComboBox* createFuelCountCombo(QWidget* parent, const std::vector<int>& counts) {
    auto* combo = new QComboBox(parent);
    for (int count : counts) {
        combo->addItem(QString::number(count), count);
    }
    combo->setMinimumWidth(84);
    return combo;
}

QWidget* createLegendItem(const QString& text, const QColor& color, QWidget* parent) {
    auto* item = new QWidget(parent);
    auto* layout = new QHBoxLayout(item);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    auto* swatch = new QFrame(item);
    swatch->setFixedSize(18, 18);
    swatch->setFrameShape(QFrame::Box);
    swatch->setStyleSheet(QString("background:%1; border:1px solid #4b5563;").arg(color.name()));

    auto* label = new QLabel(text, item);
    label->setMinimumWidth(56);

    layout->addWidget(swatch);
    layout->addWidget(label);
    return item;
}

QString fromUtf8String(const std::string& text) {
    return QString::fromUtf8(text.data(), static_cast<qsizetype>(text.size()));
}

QString replacementFailureMessage(const QString& reason) {
    return QString::fromUtf8("此燃料无法满足反应堆运行要求！原因：\n%1").arg(reason);
}

QString simulationFailureReason(const ncfr::Grid& grid, const ncfr::FuelSimulation& sim) {
    QStringList reasons;

    if (sim.fuelCells <= 0) {
        reasons << QString::fromUtf8("未检测到可运行的燃料单元。");
    } else if (sim.runningCells < sim.fuelCells) {
        reasons << QString::fromUtf8("有燃料单元未达到临界或未能运行（运行 %1/%2）。")
                       .arg(sim.runningCells)
                       .arg(sim.fuelCells);
    }

    int disconnectedHeatingClusters = 0;
    for (const ncfr::ClusterStats& cluster : sim.clusters) {
        if (cluster.rawHeating > 0 && !cluster.connectedToWall) {
            ++disconnectedHeatingClusters;
        }
    }
    if (disconnectedHeatingClusters > 0) {
        reasons << QString::fromUtf8("存在未连接到外壳的发热集群（%1 个）。")
                       .arg(disconnectedHeatingClusters);
    }

    if (sim.minClusterMargin < 0) {
        reasons << QString::fromUtf8("散热余量不足（最小散热余量 %1 H/t）。")
                       .arg(static_cast<qlonglong>(sim.minClusterMargin));
    }

    if (sim.disconnectedFunctionalBlocks > 0) {
        reasons << QString::fromUtf8("存在未接入发热集群的有效功能块（%1 个）。")
                       .arg(sim.disconnectedFunctionalBlocks);
    }

    if (sim.fluxByIndex.size() < static_cast<size_t>(grid.volume()) ||
        sim.functionalCells.size() < static_cast<size_t>(grid.volume())) {
        reasons << QString::fromUtf8("模拟结果不完整，无法确认所有燃料单元的通量状态。");
    } else {
        int unsafeFluxCells = 0;
        QStringList examples;
        for (const ncfr::Pos& pos : grid.interiorPositions()) {
            const int idx = grid.index(pos.x, pos.y, pos.z);
            const ncfr::Block& block = grid.atIndex(idx);
            if (block.kind != ncfr::BlockKind::FuelCell || block.type < 0 ||
                block.type >= static_cast<int>(ncfr::fuels().size())) {
                continue;
            }

            const ncfr::Fuel& fuel = ncfr::fuels().at(static_cast<size_t>(block.type));
            const double flux = sim.fluxByIndex.at(static_cast<size_t>(idx));
            const double limit = 2.0 * fuel.criticality;
            if (flux > limit + 1e-9) {
                ++unsafeFluxCells;
                if (examples.size() < 3) {
                    examples << QString::fromUtf8("%1 @ x=%2, y=%3, z=%4：通量 %5 > 上限 %6")
                                    .arg(fromUtf8String(fuel.nameZh))
                                    .arg(pos.x)
                                    .arg(pos.y)
                                    .arg(pos.z)
                                    .arg(flux, 0, 'f', 0)
                                    .arg(limit, 0, 'f', 0);
                }
            }
        }
        if (unsafeFluxCells > 0) {
            QString reason = QString::fromUtf8("中子通量超过 2 倍临界因子的安全上限（%1 个燃料单元）。")
                                 .arg(unsafeFluxCells);
            if (!examples.isEmpty()) {
                reason += QString::fromUtf8("\n") + examples.join(QString::fromUtf8("\n"));
            }
            reasons << reason;
        }
    }

    if (reasons.isEmpty() && !sim.compatible) {
        reasons << QString::fromUtf8("模拟结果不满足反应堆兼容运行判定。");
    }
    if (reasons.isEmpty()) {
        reasons << QString::fromUtf8("模拟结果不满足安全运行判定。");
    }

    return reasons.join(QString::fromUtf8("\n"));
}

void clearLayout(QLayout* layout) {
    while (QLayoutItem* item = layout->takeAt(0)) {
        if (QWidget* widget = item->widget()) {
            widget->deleteLater();
        }
        delete item;
    }
}

QStringList fuelFamilies() {
    QStringList families;
    std::set<std::string> seen;
    for (const ncfr::Fuel& fuel : ncfr::fuels()) {
        if (seen.insert(fuel.familyZh).second) {
            families << fromUtf8String(fuel.familyZh);
        }
    }
    return families;
}

int countFunctionalIrradiators(const ncfr::FuelSimulation& sim) {
    return static_cast<int>(std::count(sim.functionalIrradiators.begin(), sim.functionalIrradiators.end(), true));
}

double totalIrradiatorFlux(const ncfr::FuelSimulation& sim) {
    double total = 0.0;
    for (double flux : sim.irradiatorFluxByIndex) {
        total += flux;
    }
    return total;
}

int countUsefulBlocks(const ncfr::Grid& grid) {
    int count = 0;
    for (const ncfr::Pos& pos : grid.interiorPositions()) {
        if (ncfr::isFunctionalInterior(grid.at(pos.x, pos.y, pos.z).kind)) {
            ++count;
        }
    }
    return count;
}

std::vector<int> fuelIndicesInGrid(const ncfr::Grid& grid) {
    std::vector<int> indices;
    for (const ncfr::Pos& pos : grid.interiorPositions()) {
        const ncfr::Block& block = grid.at(pos.x, pos.y, pos.z);
        if (block.kind == ncfr::BlockKind::FuelCell && block.type >= 0) {
            indices.push_back(block.type);
        }
    }
    return indices;
}

std::vector<int> uniqueFuelIndices(const std::vector<int>& indices) {
    std::vector<int> unique;
    unique.reserve(indices.size());
    for (int fuelIndex : indices) {
        if (std::find(unique.begin(), unique.end(), fuelIndex) == unique.end()) {
            unique.push_back(fuelIndex);
        }
    }
    return unique;
}

std::vector<ncfr::Pos> fuelCellPortPositions(const ncfr::Grid& grid) {
    std::vector<ncfr::Pos> positions;
    positions.reserve(static_cast<size_t>(grid.volume()));
    auto appendIfCasing = [&](int x, int y, int z) {
        if (grid.isBoundary(x, y, z) && grid.at(x, y, z).kind == ncfr::BlockKind::Casing) {
            positions.push_back({x, y, z});
        }
    };

    for (int z = 1; z < grid.depth() - 1; ++z) {
        for (int y = 1; y < grid.height() - 1; ++y) {
            appendIfCasing(0, y, z);
            appendIfCasing(grid.width() - 1, y, z);
        }
    }
    for (int z = 1; z < grid.depth() - 1; ++z) {
        for (int x = 1; x < grid.width() - 1; ++x) {
            appendIfCasing(x, 0, z);
            appendIfCasing(x, grid.height() - 1, z);
        }
    }
    for (int y = 1; y < grid.height() - 1; ++y) {
        for (int x = 1; x < grid.width() - 1; ++x) {
            appendIfCasing(x, y, 0);
            appendIfCasing(x, y, grid.depth() - 1);
        }
    }
    return positions;
}

void clearFuelCellPorts(ncfr::Grid& grid) {
    for (int z = 0; z < grid.depth(); ++z) {
        for (int y = 0; y < grid.height(); ++y) {
            for (int x = 0; x < grid.width(); ++x) {
                ncfr::Block& block = grid.at(x, y, z);
                if (grid.isBoundary(x, y, z) && block.kind == ncfr::BlockKind::CellPort) {
                    block = {ncfr::BlockKind::Casing, -1};
                }
            }
        }
    }
}

bool rebuildFuelCellPorts(ncfr::Grid& grid, const std::vector<int>& fuelIndices) {
    clearFuelCellPorts(grid);
    const std::vector<int> unique = uniqueFuelIndices(fuelIndices);
    const std::vector<ncfr::Pos> positions = fuelCellPortPositions(grid);
    if (positions.size() < unique.size() * 2) {
        return false;
    }

    size_t positionIndex = 0;
    for (int fuelIndex : unique) {
        const ncfr::Pos inputPos = positions.at(positionIndex++);
        grid.at(inputPos.x, inputPos.y, inputPos.z) =
            {ncfr::BlockKind::CellPort, ncfr::fuelCellPortType(fuelIndex, ncfr::FuelCellPortRole::Input)};

        const ncfr::Pos outputPos = positions.at(positionIndex++);
        grid.at(outputPos.x, outputPos.y, outputPos.z) =
            {ncfr::BlockKind::CellPort, ncfr::fuelCellPortType(fuelIndex, ncfr::FuelCellPortRole::Output)};
    }
    return true;
}

QString blockKindKey(ncfr::BlockKind kind) {
    switch (kind) {
    case ncfr::BlockKind::Empty:
        return QStringLiteral("empty");
    case ncfr::BlockKind::Casing:
        return QStringLiteral("casing");
    case ncfr::BlockKind::Controller:
        return QStringLiteral("controller");
    case ncfr::BlockKind::CellPort:
        return QStringLiteral("cell_port");
    case ncfr::BlockKind::IrradiatorPort:
        return QStringLiteral("irradiator_port");
    case ncfr::BlockKind::VentIn:
        return QStringLiteral("vent_in");
    case ncfr::BlockKind::VentOut:
        return QStringLiteral("vent_out");
    case ncfr::BlockKind::Source:
        return QStringLiteral("source");
    case ncfr::BlockKind::FuelCell:
        return QStringLiteral("fuel_cell");
    case ncfr::BlockKind::Moderator:
        return QStringLiteral("moderator");
    case ncfr::BlockKind::Reflector:
        return QStringLiteral("reflector");
    case ncfr::BlockKind::Shield:
        return QStringLiteral("shield");
    case ncfr::BlockKind::Irradiator:
        return QStringLiteral("irradiator");
    case ncfr::BlockKind::Sink:
        return QStringLiteral("sink");
    }
    return QStringLiteral("unknown");
}

QJsonObject sizeToJson(const ncfr::Grid& grid) {
    return {
        {QStringLiteral("length"), grid.internalA()},
        {QStringLiteral("width"), grid.internalB()},
        {QStringLiteral("height"), grid.internalC()},
    };
}

QJsonObject fullSizeToJson(const ncfr::Grid& grid) {
    return {
        {QStringLiteral("width"), grid.width()},
        {QStringLiteral("height"), grid.height()},
        {QStringLiteral("depth"), grid.depth()},
    };
}

QJsonObject fuelToJson(int fuelIndex) {
    const ncfr::Fuel& fuel = ncfr::fuels().at(static_cast<size_t>(fuelIndex));
    return {
        {QStringLiteral("index"), fuelIndex},
        {QStringLiteral("familyZh"), fromUtf8String(fuel.familyZh)},
        {QStringLiteral("familyEn"), fromUtf8String(fuel.familyEn)},
        {QStringLiteral("code"), fromUtf8String(fuel.code)},
        {QStringLiteral("formZh"), fromUtf8String(fuel.formZh)},
        {QStringLiteral("nameZh"), fromUtf8String(fuel.nameZh)},
        {QStringLiteral("nameEn"), fromUtf8String(fuel.nameEn)},
        {QStringLiteral("time"), fuel.time},
        {QStringLiteral("heat"), fuel.heat},
        {QStringLiteral("efficiency"), fuel.efficiency},
        {QStringLiteral("criticality"), fuel.criticality},
        {QStringLiteral("intrinsicFlux"), fuel.intrinsicFlux},
        {QStringLiteral("decayFactor"), fuel.decayFactor},
        {QStringLiteral("selfPriming"), fuel.selfPriming},
    };
}

QJsonObject typedBlockDataToJson(const ncfr::Block& block) {
    QJsonObject typedData;
    if (block.type < 0) {
        return typedData;
    }

    switch (block.kind) {
    case ncfr::BlockKind::Source:
        break;
    case ncfr::BlockKind::CellPort: {
        const int fuelIndex = ncfr::fuelCellPortFuelIndex(block.type);
        if (fuelIndex < 0 || fuelIndex >= static_cast<int>(ncfr::fuels().size())) {
            break;
        }
        const ncfr::Fuel& fuel = ncfr::fuels().at(static_cast<size_t>(fuelIndex));
        const ncfr::FuelCellPortRole role = ncfr::fuelCellPortRole(block.type);
        typedData.insert(QStringLiteral("fuelIndex"), fuelIndex);
        typedData.insert(QStringLiteral("familyZh"), fromUtf8String(fuel.familyZh));
        typedData.insert(QStringLiteral("familyEn"), fromUtf8String(fuel.familyEn));
        typedData.insert(QStringLiteral("code"), fromUtf8String(fuel.code));
        typedData.insert(QStringLiteral("fuelNameZh"), fromUtf8String(fuel.nameZh));
        typedData.insert(QStringLiteral("fuelNameEn"), fromUtf8String(fuel.nameEn));
        typedData.insert(QStringLiteral("role"), role == ncfr::FuelCellPortRole::Output
                                                   ? QStringLiteral("output")
                                                   : QStringLiteral("input"));
        typedData.insert(QStringLiteral("roleZh"),
                         QString::fromUtf8(ncfr::fuelCellPortRoleNameZh(role)));
        break;
    }
    case ncfr::BlockKind::IrradiatorPort: {
        const ncfr::FuelCellPortRole role = ncfr::irradiatorPortRole(block.type);
        typedData.insert(QStringLiteral("role"), role == ncfr::FuelCellPortRole::Output
                                                   ? QStringLiteral("output")
                                                   : QStringLiteral("input"));
        typedData.insert(QStringLiteral("roleZh"),
                         QString::fromUtf8(ncfr::fuelCellPortRoleNameZh(role)));
        break;
    }
    case ncfr::BlockKind::FuelCell: {
        const ncfr::Fuel& fuel = ncfr::fuels().at(static_cast<size_t>(block.type));
        typedData.insert(QStringLiteral("index"), block.type);
        typedData.insert(QStringLiteral("familyZh"), fromUtf8String(fuel.familyZh));
        typedData.insert(QStringLiteral("familyEn"), fromUtf8String(fuel.familyEn));
        typedData.insert(QStringLiteral("code"), fromUtf8String(fuel.code));
        typedData.insert(QStringLiteral("formZh"), fromUtf8String(fuel.formZh));
        typedData.insert(QStringLiteral("nameZh"), fromUtf8String(fuel.nameZh));
        typedData.insert(QStringLiteral("nameEn"), fromUtf8String(fuel.nameEn));
        typedData.insert(QStringLiteral("heat"), fuel.heat);
        typedData.insert(QStringLiteral("criticality"), fuel.criticality);
        typedData.insert(QStringLiteral("selfPriming"), fuel.selfPriming);
        break;
    }
    case ncfr::BlockKind::Moderator: {
        const ncfr::ModeratorType& moderator = ncfr::moderatorTypes().at(static_cast<size_t>(block.type));
        typedData.insert(QStringLiteral("registryName"), fromUtf8String(moderator.registryName));
        typedData.insert(QStringLiteral("nameZh"), fromUtf8String(moderator.nameZh));
        typedData.insert(QStringLiteral("nameEn"), fromUtf8String(moderator.nameEn));
        typedData.insert(QStringLiteral("fluxFactor"), moderator.fluxFactor);
        typedData.insert(QStringLiteral("efficiency"), moderator.efficiency);
        break;
    }
    case ncfr::BlockKind::Reflector: {
        const ncfr::ReflectorType& reflector = ncfr::reflectorTypes().at(static_cast<size_t>(block.type));
        typedData.insert(QStringLiteral("registryName"), fromUtf8String(reflector.registryName));
        typedData.insert(QStringLiteral("nameZh"), fromUtf8String(reflector.nameZh));
        typedData.insert(QStringLiteral("nameEn"), fromUtf8String(reflector.nameEn));
        typedData.insert(QStringLiteral("efficiency"), reflector.efficiency);
        typedData.insert(QStringLiteral("reflectivity"), reflector.reflectivity);
        break;
    }
    case ncfr::BlockKind::Shield: {
        const ncfr::ShieldType& shield = ncfr::shieldTypes().at(static_cast<size_t>(block.type));
        typedData.insert(QStringLiteral("registryName"), fromUtf8String(shield.registryName));
        typedData.insert(QStringLiteral("nameZh"), fromUtf8String(shield.nameZh));
        typedData.insert(QStringLiteral("nameEn"), fromUtf8String(shield.nameEn));
        typedData.insert(QStringLiteral("heatPerFlux"), shield.heatPerFlux);
        typedData.insert(QStringLiteral("efficiency"), shield.efficiency);
        break;
    }
    case ncfr::BlockKind::Irradiator: {
        const ncfr::IrradiatorRecipeType& recipe = ncfr::irradiatorRecipeTypes().at(static_cast<size_t>(block.type));
        typedData.insert(QStringLiteral("registryName"), QStringLiteral("nuclearcraft:fission_irradiator"));
        typedData.insert(QStringLiteral("inputName"), fromUtf8String(recipe.inputName));
        typedData.insert(QStringLiteral("outputName"), fromUtf8String(recipe.outputName));
        typedData.insert(QStringLiteral("nameZh"), fromUtf8String(recipe.nameZh));
        typedData.insert(QStringLiteral("nameEn"), fromUtf8String(recipe.nameEn));
        typedData.insert(QStringLiteral("heatPerFlux"), recipe.heatPerFlux);
        typedData.insert(QStringLiteral("efficiency"), recipe.efficiency);
        break;
    }
    case ncfr::BlockKind::Sink: {
        const ncfr::SinkType& sink = ncfr::sinkTypes().at(static_cast<size_t>(block.type));
        typedData.insert(QStringLiteral("index"), sink.index);
        typedData.insert(QStringLiteral("ruleId"), fromUtf8String(sink.ruleId));
        typedData.insert(QStringLiteral("sourceName"), fromUtf8String(sink.sourceName));
        typedData.insert(QStringLiteral("nameZh"), fromUtf8String(sink.nameZh));
        typedData.insert(QStringLiteral("nameEn"), fromUtf8String(sink.nameEn));
        typedData.insert(QStringLiteral("cooling"), sink.cooling);
        typedData.insert(QStringLiteral("rule"), fromUtf8String(sink.rule));
        break;
    }
    default:
        break;
    }
    return typedData;
}

QJsonObject blockToJson(const ncfr::Grid& grid, int x, int y, int z) {
    const ncfr::Block& block = grid.at(x, y, z);
    QJsonObject object{
        {QStringLiteral("x"), x},
        {QStringLiteral("y"), y},
        {QStringLiteral("z"), z},
        {QStringLiteral("kind"), blockKindKey(block.kind)},
        {QStringLiteral("type"), block.type},
        {QStringLiteral("displayName"), fromUtf8String(ncfr::blockDisplayName(block))},
    };

    QJsonObject typedData = typedBlockDataToJson(block);
    if (!typedData.isEmpty()) {
        object.insert(QStringLiteral("data"), typedData);
    }
    return object;
}

QJsonArray selectedFuelsToJson(const ncfr::BuildRequest& request) {
    QJsonArray array;
    for (int fuelIndex : request.fuelIndices) {
        array.append(fuelToJson(fuelIndex));
    }
    return array;
}

QJsonObject requestToJson(const ncfr::BuildRequest& request) {
    return {
        {QStringLiteral("fuelCells"), selectedFuelsToJson(request)},
        {QStringLiteral("sourceCount"), ncfr::requiredSourceCount(request)},
        {QStringLiteral("disableCaliforniumNeutronReflector"), request.disableCaliforniumNeutronReflector},
    };
}

QJsonObject gridToJson(const ncfr::Grid& grid) {
    QJsonArray layers;
    for (int z = 0; z < grid.depth(); ++z) {
        QJsonArray rows;
        for (int y = 0; y < grid.height(); ++y) {
            QJsonArray row;
            for (int x = 0; x < grid.width(); ++x) {
                row.append(blockToJson(grid, x, y, z));
            }
            rows.append(row);
        }
        layers.append(QJsonObject{
            {QStringLiteral("z"), z},
            {QStringLiteral("rows"), rows},
        });
    }

    return {
        {QStringLiteral("coordinateSystem"),
         QString::fromUtf8("外部网格零基坐标；边界坐标为外壳，内部坐标范围为 1..内部尺寸。")},
        {QStringLiteral("layers"), layers},
    };
}

QJsonDocument resultToJsonDocument(const ncfr::OptimizationResult& result) {
    QJsonObject metrics{
        {QStringLiteral("minCoolingMargin"), static_cast<qint64>(result.minCoolingMargin)},
        {QStringLiteral("usefulBlocks"), result.usefulBlocks},
        {QStringLiteral("disconnectedFunctionalBlocks"), result.disconnectedFunctionalBlocks},
        {QStringLiteral("functionalIrradiators"), result.functionalIrradiators},
        {QStringLiteral("irradiatorFlux"), result.irradiatorFlux},
    };

    return QJsonDocument(QJsonObject{
        {QStringLiteral("schema"), QStringLiteral("nuclearcraft-fission-reactor-result")},
        {QStringLiteral("schemaVersion"), 4},
        {QStringLiteral("request"), requestToJson(result.request)},
        {QStringLiteral("internalSize"), sizeToJson(result.grid)},
        {QStringLiteral("externalSize"), fullSizeToJson(result.grid)},
        {QStringLiteral("metrics"), metrics},
        {QStringLiteral("grid"), gridToJson(result.grid)},
    });
}

QString findProjectDirectoryFrom(const QString& startPath) {
    QDir dir(startPath);
    if (QFileInfo(startPath).isFile()) {
        dir = QFileInfo(startPath).absoluteDir();
    }

    while (true) {
        if (QFileInfo::exists(dir.filePath(QStringLiteral("CMakeLists.txt")))) {
            return dir.absolutePath();
        }
        if (!dir.cdUp()) {
            return {};
        }
    }
}

QString findProjectDirectory() {
    const QString fromCurrentPath = findProjectDirectoryFrom(QDir::currentPath());
    if (!fromCurrentPath.isEmpty()) {
        return fromCurrentPath;
    }
    return findProjectDirectoryFrom(QCoreApplication::applicationDirPath());
}

QString nextResultFilePath(const QString& directoryPath) {
    QDir dir(directoryPath);
    for (int i = 0; ; ++i) {
        const QString path = dir.filePath(QString::fromUtf8("Result %1.json").arg(i));
        if (!QFileInfo::exists(path)) {
            return path;
        }
    }
}

class GenerationDialog : public QDialog {
public:
    explicit GenerationDialog(std::function<void()> cancelHandler, QWidget* parent)
        : QDialog(parent), cancelHandler_(std::move(cancelHandler)) {
        setWindowTitle(QString::fromUtf8("请稍候"));
        setWindowModality(Qt::ApplicationModal);
        setWindowFlags(windowFlags() | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowCloseButtonHint);

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(24, 22, 24, 22);
        layout->setSpacing(14);

        label_ = new QLabel(QString::fromUtf8("正在生成方案中"), this);
        label_->setAlignment(Qt::AlignCenter);
        layout->addWidget(label_);

        auto* progress = new QProgressBar(this);
        progress->setRange(0, 0);
        progress->setTextVisible(false);
        progress->setMinimumWidth(280);
        layout->addWidget(progress);

        cancelButton_ = new QPushButton(QString::fromUtf8("取消"), this);
        layout->addWidget(cancelButton_);
        connect(cancelButton_, &QPushButton::clicked, this, [this]() {
            requestCancel();
        });

        setFixedSize(sizeHint());
    }

    void requestCancel() {
        if (cancelRequested_) {
            return;
        }
        cancelRequested_ = true;
        if (label_ != nullptr) {
            label_->setText(QString::fromUtf8("正在取消计算，请稍候"));
        }
        if (cancelButton_ != nullptr) {
            cancelButton_->setEnabled(false);
        }
        if (cancelHandler_) {
            cancelHandler_();
        }
    }

    void reject() override {
        requestCancel();
    }

protected:
    void closeEvent(QCloseEvent* event) override {
        requestCancel();
        event->ignore();
    }

private:
    std::function<void()> cancelHandler_;
    QLabel* label_ = nullptr;
    QPushButton* cancelButton_ = nullptr;
    bool cancelRequested_ = false;
};

} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(QString::fromUtf8("裂变反应堆搭建优化器"));
    resize(1180, 760);

    auto* central = new QWidget(this);
    auto* rootLayout = new QVBoxLayout(central);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(10);

    rootLayout->addWidget(createInputPanel());
    rootLayout->addWidget(createGridPanel(), 1);

    setCentralWidget(central);
    statusBar()->showMessage(QString::fromUtf8("就绪"));

    connect(exportJsonButton_, &QPushButton::clicked, this, [this]() { exportResultAsJson(); });
    connect(layerSpin_, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int layer) {
        showLayer(layer - 1);
    });
    connect(gridWidget_, &ReactorGridWidget::fuelCellClicked, this,
            [this](int x, int y, int z, int index) {
                showFuelReplacementCandidates(x, y, z, index);
            });

    clearResultView();

    QTimer::singleShot(0, this, [this]() {
        QMessageBox::information(
            this,
            QString::fromUtf8("注意事项"),
            QString::fromUtf8(
                "请在使用本工具前仔细阅读以下注意事项：\n"
                "0.本工具分为两个功能：\n"
                "   普通结构生成:选择燃料单元的数量和燃料种类，输出可行的反应堆设计方案。\n"
                "   辐照结构生成:选择六种燃料，生成带辐照仓且运行速率最大的反应堆设计方案。\n"
                "1.中子源由非自启动燃料单元自动决定，生成方案统一显示为“任意中子源”。\n"
                "2.普通结构生成支持 1、2、4 个燃料单元；辐照结构生成固定为一个中心辐照仓和六个燃料单元。\n"
                "3.两个生成区域互相独立，每次点击一个区域内的生成方案按钮。\n"
                "4.在生成的结构内点击燃料单元，可以将此燃料替换成其他燃料，替换失败会说明原因。\n"
                "5.使用过程产生任何问题请在贪婪2交流1、4群@Atopts.\n"));
    });
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (generationThread_ != nullptr) {
        event->ignore();
        requestCancelGeneration();
        if (generationDialog_ != nullptr) {
            generationDialog_->raise();
            generationDialog_->activateWindow();
        }
        return;
    }
    QMainWindow::closeEvent(event);
}

QWidget* MainWindow::createInputPanel() {
    auto* panel = new QWidget(this);
    auto* rootLayout = new QVBoxLayout(panel);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(10);

    auto* inputLayout = new QHBoxLayout();
    inputLayout->setSpacing(10);
    inputLayout->addWidget(createFuelInputPanel(QString::fromUtf8("普通结构生成"), normalInput_, panel, {1, 2, 4}, 0));
    inputLayout->addWidget(createFuelInputPanel(QString::fromUtf8("辐照结构生成"), irradiatorInput_, panel, {}, 6));
    rootLayout->addLayout(inputLayout);

    auto* exportLayout = new QHBoxLayout();
    exportLayout->addStretch(1);
    exportJsonButton_ = new QPushButton(QString::fromUtf8("将结果导出为JSON格式"), panel);
    exportJsonButton_->setMinimumWidth(180);
    exportJsonButton_->setEnabled(false);
    exportLayout->addWidget(exportJsonButton_);
    rootLayout->addLayout(exportLayout);

    rebuildFuelRows(normalInput_);
    rebuildFuelRows(irradiatorInput_);
    return panel;
}

QWidget* MainWindow::createFuelInputPanel(const QString& title, FuelInputControls& controls,
                                          QWidget* parent, const std::vector<int>& selectableFuelCounts,
                                          int fixedFuelCellCount) {
    auto* group = new QGroupBox(title, parent);
    auto* rootLayout = new QVBoxLayout(group);
    rootLayout->setSpacing(8);

    controls.fixedFuelCellCount = fixedFuelCellCount;

    auto* topLayout = new QHBoxLayout();
    topLayout->addWidget(new QLabel(QString::fromUtf8("燃料单元"), group));
    if (fixedFuelCellCount > 0) {
        auto* countLabel = new QLabel(QString::number(fixedFuelCellCount), group);
        countLabel->setMinimumWidth(84);
        topLayout->addWidget(countLabel);
    } else {
        controls.fuelCellCountCombo = createFuelCountCombo(group, selectableFuelCounts);
        topLayout->addWidget(controls.fuelCellCountCombo);
        FuelInputControls* controlsPtr = &controls;
        connect(controls.fuelCellCountCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this, controlsPtr]() {
                    rebuildFuelRows(*controlsPtr);
                });
    }
    topLayout->addStretch(1);

    controls.generateButton = new QPushButton(QString::fromUtf8("生成方案"), group);
    controls.generateButton->setMinimumWidth(112);
    topLayout->addWidget(controls.generateButton);
    rootLayout->addLayout(topLayout);

    if (fixedFuelCellCount == 6) {
        controls.disableCaliforniumReflectorCheck =
            new QCheckBox(QString::fromUtf8("禁用锎中子反射器"), group);
        controls.disableCaliforniumReflectorCheck->setToolTip(
            QString::fromUtf8("勾选后，辐照结构生成不会使用锎中子反射器（增殖器）。"));
        rootLayout->addWidget(controls.disableCaliforniumReflectorCheck);
    } else {
        controls.disableCaliforniumReflectorCheck = nullptr;
    }

    auto* fuelGroup = new QGroupBox(QString::fromUtf8("燃料单元"), group);
    controls.fuelRowsLayout = new QVBoxLayout(fuelGroup);
    controls.fuelRowsLayout->setSpacing(6);
    rootLayout->addWidget(fuelGroup);

    FuelInputControls* controlsPtr = &controls;
    connect(controls.generateButton, &QPushButton::clicked, this, [this, controlsPtr]() {
        generateLayout(*controlsPtr);
    });
    return group;
}

QWidget* MainWindow::createGridPanel() {
    auto* group = new QGroupBox(QString::fromUtf8("二维分层方案"), this);
    auto* rootLayout = new QHBoxLayout(group);
    rootLayout->setSpacing(10);

    rootLayout->addWidget(createFuelReplacementPanel());

    auto* rightPanel = new QWidget(group);
    auto* layout = new QVBoxLayout(rightPanel);
    layout->setContentsMargins(0, 0, 0, 0);

    auto* topLayout = new QHBoxLayout();
    topLayout->addWidget(new QLabel(QString::fromUtf8("层"), group));
    layerSpin_ = new QSpinBox(group);
    layerSpin_->setRange(1, 1);
    layerSpin_->setEnabled(false);
    layerSpin_->setMinimumWidth(80);
    topLayout->addWidget(layerSpin_);
    topLayout->addStretch(1);
    layout->addLayout(topLayout);

    gridWidget_ = new ReactorGridWidget(group);
    layout->addWidget(gridWidget_, 1);
    layout->addWidget(createLegendPanel());

    rootLayout->addWidget(rightPanel, 1);
    return group;
}

QWidget* MainWindow::createFuelReplacementPanel() {
    auto* group = new QGroupBox(QString::fromUtf8("可替换燃料"), this);
    auto* layout = new QVBoxLayout(group);
    layout->setSpacing(8);

    fuelReplacementTitle_ = new QLabel(QString::fromUtf8(""), group);
    fuelReplacementTitle_->setWordWrap(true);
    layout->addWidget(fuelReplacementTitle_);

    fuelReplacementList_ = new QListWidget(group);
    fuelReplacementList_->setSelectionMode(QAbstractItemView::SingleSelection);
    fuelReplacementList_->setMinimumWidth(260);
    layout->addWidget(fuelReplacementList_, 1);

    connect(fuelReplacementList_, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) {
        replaceSelectedFuel(item);
    });

    fuelReplacementPanel_ = group;
    fuelReplacementPanel_->hide();
    return group;
}

QWidget* MainWindow::createLegendPanel() {
    auto* group = new QGroupBox(QString::fromUtf8("图例"), this);
    auto* layout = new QHBoxLayout(group);
    layout->setSpacing(12);

    const QList<QPair<QString, ncfr::BlockKind>> entries = {
        {QString::fromUtf8("空"), ncfr::BlockKind::Empty},
        {QString::fromUtf8("外壳"), ncfr::BlockKind::Casing},
        {QString::fromUtf8("控制器"), ncfr::BlockKind::Controller},
        {QString::fromUtf8("燃料单元端口"), ncfr::BlockKind::CellPort},
        {QString::fromUtf8("辐照器端口"), ncfr::BlockKind::IrradiatorPort},
        {QString::fromUtf8("通风口"), ncfr::BlockKind::VentIn},
        {QString::fromUtf8("中子源"), ncfr::BlockKind::Source},
        {QString::fromUtf8("燃料"), ncfr::BlockKind::FuelCell},
        {QString::fromUtf8("减速剂"), ncfr::BlockKind::Moderator},
        {QString::fromUtf8("反射器"), ncfr::BlockKind::Reflector},
        {QString::fromUtf8("裂变中子辐照器"), ncfr::BlockKind::Irradiator},
        {QString::fromUtf8("散热器"), ncfr::BlockKind::Sink},
    };

    for (const auto& entry : entries) {
        layout->addWidget(createLegendItem(entry.first, ReactorGridWidget::colorForKind(entry.second), group));
    }
    layout->addStretch(1);

    return group;
}

void MainWindow::rebuildFuelRows(FuelInputControls& controls) {
    if (controls.fuelRowsLayout == nullptr) {
        return;
    }

    clearLayout(controls.fuelRowsLayout);
    controls.fuelFamilyCombos.clear();
    controls.fuelCombos.clear();

    const QStringList families = fuelFamilies();
    const int count = controls.fixedFuelCellCount > 0
        ? controls.fixedFuelCellCount
        : (controls.fuelCellCountCombo != nullptr ? controls.fuelCellCountCombo->currentData().toInt() : 0);
    controls.fuelFamilyCombos.reserve(static_cast<size_t>(count));
    controls.fuelCombos.reserve(static_cast<size_t>(count));
    FuelInputControls* controlsPtr = &controls;
    for (int row = 0; row < count; ++row) {
        auto* rowWidget = new QWidget(this);
        auto* rowLayout = new QHBoxLayout(rowWidget);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(6);

        auto* familyCombo = new QComboBox(rowWidget);
        familyCombo->addItems(families);
        familyCombo->setMinimumWidth(92);

        auto* fuelCombo = new QComboBox(rowWidget);
        fuelCombo->setMinimumWidth(260);

        rowLayout->addWidget(new QLabel(QString::fromUtf8("燃料 %1").arg(row + 1), rowWidget));
        rowLayout->addWidget(familyCombo);
        rowLayout->addWidget(fuelCombo, 1);
        controls.fuelRowsLayout->addWidget(rowWidget);

        controls.fuelFamilyCombos.push_back(familyCombo);
        controls.fuelCombos.push_back(fuelCombo);
        connect(familyCombo, &QComboBox::currentTextChanged, this, [this, controlsPtr, row]() {
            updateFuelComboForRow(*controlsPtr, row);
        });
        if (!families.isEmpty()) {
            familyCombo->setCurrentIndex(row % families.size());
        }
        updateFuelComboForRow(controls, row);
    }
    controls.fuelRowsLayout->addStretch(1);
}

void MainWindow::updateFuelComboForRow(FuelInputControls& controls, int row) {
    if (row < 0 || row >= static_cast<int>(controls.fuelFamilyCombos.size()) ||
        row >= static_cast<int>(controls.fuelCombos.size())) {
        return;
    }

    QComboBox* familyCombo = controls.fuelFamilyCombos.at(static_cast<size_t>(row));
    QComboBox* fuelCombo = controls.fuelCombos.at(static_cast<size_t>(row));
    const QString family = familyCombo->currentText();
    fuelCombo->clear();
    for (int i = 0; i < static_cast<int>(ncfr::fuels().size()); ++i) {
        const ncfr::Fuel& fuel = ncfr::fuels().at(static_cast<size_t>(i));
        if (fromUtf8String(fuel.familyZh) == family) {
            fuelCombo->addItem(fromUtf8String(fuel.nameZh), i);
        }
    }
}

void MainWindow::setGenerationButtonsEnabled(bool enabled) {
    if (normalInput_.generateButton != nullptr) {
        normalInput_.generateButton->setEnabled(enabled);
    }
    if (irradiatorInput_.generateButton != nullptr) {
        irradiatorInput_.generateButton->setEnabled(enabled);
    }
}

ncfr::BuildRequest MainWindow::buildRequestFromUi(const FuelInputControls& controls) const {
    ncfr::BuildRequest request;
    for (QComboBox* combo : controls.fuelCombos) {
        if (combo == nullptr || combo->currentIndex() < 0) {
            throw std::runtime_error("请选择每个燃料单元的燃料。");
        }
        request.fuelIndices.push_back(combo->currentData().toInt());
    }
    if (controls.disableCaliforniumReflectorCheck != nullptr) {
        request.disableCaliforniumNeutronReflector =
            controls.disableCaliforniumReflectorCheck->isChecked();
    }
    return request;
}

void MainWindow::generateLayout(const FuelInputControls& controls) {
    if (generationThread_ != nullptr) {
        return;
    }

    ncfr::BuildRequest request;
    try {
        request = buildRequestFromUi(controls);
    } catch (const std::exception& ex) {
        QMessageBox::warning(this, QString::fromUtf8("输入不完整"), QString::fromUtf8(ex.what()));
        return;
    }
    setGenerationButtonsEnabled(false);
    exportJsonButton_->setEnabled(false);
    statusBar()->showMessage(QString::fromUtf8("正在生成方案中"));
    QApplication::setOverrideCursor(Qt::WaitCursor);
    generationCancelFlag_ = std::make_shared<std::atomic_bool>(false);
    std::shared_ptr<std::atomic_bool> cancelFlag = generationCancelFlag_;
    showGenerationDialog();

    QThread* thread = QThread::create([this, request = std::move(request), cancelFlag]() {
        try {
            if (ncfr::fuels().size() != 135 || ncfr::sinkTypes().size() != 86 ||
                ncfr::moderatorTypes().size() != 6 || ncfr::reflectorTypes().size() != 4 ||
                ncfr::sourceTypes().size() != 3 || ncfr::shieldTypes().size() != 2 ||
                ncfr::irradiatorRecipeTypes().size() != 3) {
                throw std::runtime_error(QString::fromUtf8("嵌入数据数量与工作表预期不一致。").toStdString());
            }

            ncfr::OptimizationResult result = ncfr::optimizeLayout(request, cancelFlag.get());
            if (cancelFlag->load()) {
                throw ncfr::OptimizationCanceled();
            }
            QMetaObject::invokeMethod(
                this,
                [this, result = std::move(result)]() mutable {
                    finishGenerationSuccess(std::move(result));
                },
                Qt::QueuedConnection);
        } catch (const ncfr::OptimizationCanceled&) {
            QMetaObject::invokeMethod(
                this,
                [this]() {
                    finishGenerationCanceled();
                },
                Qt::QueuedConnection);
        } catch (const std::exception& ex) {
            if (cancelFlag->load()) {
                QMetaObject::invokeMethod(
                    this,
                    [this]() {
                        finishGenerationCanceled();
                    },
                    Qt::QueuedConnection);
                return;
            }
            const QString message = QString::fromUtf8(ex.what());
            QMetaObject::invokeMethod(
                this,
                [this, message]() {
                    finishGenerationFailure(message);
                },
                Qt::QueuedConnection);
        } catch (...) {
            QMetaObject::invokeMethod(
                this,
                [this]() {
                    finishGenerationFailure(QString::fromUtf8("生成方案时发生未知错误。"));
                },
                Qt::QueuedConnection);
        }
    });

    generationThread_ = thread;
    connect(thread, &QThread::finished, this, [this, thread]() {
        if (generationThread_ == thread) {
            generationThread_ = nullptr;
            generationCancelFlag_.reset();
        }
        thread->deleteLater();
    });
    thread->start();
}

void MainWindow::showGenerationDialog() {
    if (generationDialog_ != nullptr) {
        generationDialog_->raise();
        generationDialog_->activateWindow();
        return;
    }

    generationDialog_ = new GenerationDialog([this]() {
        requestCancelGeneration();
    }, this);
    generationDialog_->show();
    generationDialog_->raise();
    generationDialog_->activateWindow();
}

void MainWindow::closeGenerationDialog() {
    if (generationDialog_ == nullptr) {
        return;
    }

    QDialog* dialog = generationDialog_;
    generationDialog_ = nullptr;
    dialog->accept();
    dialog->deleteLater();
}

void MainWindow::requestCancelGeneration() {
    if (generationCancelFlag_ != nullptr) {
        generationCancelFlag_->store(true);
    }
    statusBar()->showMessage(QString::fromUtf8("正在取消生成方案"));
}

void MainWindow::finishGenerationSuccess(ncfr::OptimizationResult result) {
    closeGenerationDialog();
    QApplication::restoreOverrideCursor();
    currentResult_ = std::move(result);
    updateResultView();
    statusBar()->showMessage(QString::fromUtf8("生成完成"));
    setGenerationButtonsEnabled(true);
    exportJsonButton_->setEnabled(true);
}

void MainWindow::finishGenerationCanceled() {
    closeGenerationDialog();
    QApplication::restoreOverrideCursor();
    statusBar()->showMessage(QString::fromUtf8("已取消生成"));
    setGenerationButtonsEnabled(true);
    exportJsonButton_->setEnabled(currentResult_.has_value());
}

void MainWindow::finishGenerationFailure(const QString& message) {
    closeGenerationDialog();
    QApplication::restoreOverrideCursor();
    clearResultView();
    statusBar()->showMessage(QString::fromUtf8("生成失败"));
    setGenerationButtonsEnabled(true);
    QMessageBox::critical(this, QString::fromUtf8("错误"), message);
}

void MainWindow::exportResultAsJson() {
    if (!currentResult_.has_value()) {
        exportJsonButton_->setEnabled(false);
        return;
    }

    const QString projectDirectory = findProjectDirectory();
    if (projectDirectory.isEmpty()) {
        QMessageBox::critical(this, QString::fromUtf8("导出失败"), QString::fromUtf8("未找到 CMakeLists.txt 所在目录。"));
        return;
    }

    const QString filePath = nextResultFilePath(projectDirectory);
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::NewOnly | QIODevice::Text)) {
        QMessageBox::critical(this, QString::fromUtf8("导出失败"), QString::fromUtf8("无法创建文件：%1").arg(filePath));
        return;
    }

    const QByteArray json = resultToJsonDocument(*currentResult_).toJson(QJsonDocument::Indented);
    if (file.write(json) != json.size()) {
        file.close();
        QFile::remove(filePath);
        QMessageBox::critical(this, QString::fromUtf8("导出失败"), QString::fromUtf8("写入文件失败：%1").arg(filePath));
        return;
    }

    file.close();
    statusBar()->showMessage(QString::fromUtf8("已导出：%1").arg(filePath));
    QMessageBox::information(this, QString::fromUtf8("导出完成"), QString::fromUtf8("已导出到：\n%1").arg(filePath));
}

void MainWindow::showLayer(int zeroBasedLayer) {
    if (!currentResult_.has_value()) {
        return;
    }
    gridWidget_->setLayer(zeroBasedLayer);
}

void MainWindow::showFuelReplacementCandidates(int x, int y, int z, int index) {
    if (!currentResult_.has_value() || fuelReplacementPanel_ == nullptr ||
        fuelReplacementTitle_ == nullptr || fuelReplacementList_ == nullptr) {
        return;
    }

    const ncfr::Grid& grid = currentResult_->grid;
    if (!grid.inBounds(x, y, z) || index != grid.index(x, y, z)) {
        hideFuelReplacementPanel();
        return;
    }

    const ncfr::Block& block = grid.at(x, y, z);
    if (block.kind != ncfr::BlockKind::FuelCell || block.type < 0 ||
        block.type >= static_cast<int>(ncfr::fuels().size())) {
        hideFuelReplacementPanel();
        return;
    }

    selectedFuelCell_ = ncfr::Pos{x, y, z};
    const ncfr::Fuel& currentFuel = ncfr::fuels().at(static_cast<size_t>(block.type));
    fuelReplacementTitle_->setText(QString::fromUtf8("当前燃料：%1\n坐标：x=%2, y=%3, z=%4")
                                       .arg(fromUtf8String(currentFuel.nameZh))
                                       .arg(x)
                                       .arg(y)
                                       .arg(z));

    fuelReplacementList_->clear();
    for (int i = 0; i < static_cast<int>(ncfr::fuels().size()); ++i) {
        const ncfr::Fuel& fuel = ncfr::fuels().at(static_cast<size_t>(i));
        if (fuel.selfPriming != currentFuel.selfPriming) {
            continue;
        }

        QString text = fromUtf8String(fuel.nameZh);
        if (i == block.type) {
            text += QString::fromUtf8("（当前）");
        }
        auto* item = new QListWidgetItem(text, fuelReplacementList_);
        item->setData(Qt::UserRole, i);
        item->setToolTip(QString::fromUtf8("%1\n临界因子：%2\n基础产热：%3 H/t")
                             .arg(fromUtf8String(fuel.code))
                             .arg(fuel.criticality)
                             .arg(fuel.heat));
        if (i == block.type) {
            fuelReplacementList_->setCurrentItem(item);
        }
    }

    fuelReplacementPanel_->show();
}

void MainWindow::replaceSelectedFuel(QListWidgetItem* item) {
    if (item == nullptr || !currentResult_.has_value() || !selectedFuelCell_.has_value()) {
        return;
    }

    bool ok = false;
    const int fuelIndex = item->data(Qt::UserRole).toInt(&ok);
    if (!ok || fuelIndex < 0 || fuelIndex >= static_cast<int>(ncfr::fuels().size())) {
        return;
    }

    const ncfr::Pos pos = *selectedFuelCell_;
    if (!currentResult_->grid.inBounds(pos.x, pos.y, pos.z)) {
        hideFuelReplacementPanel();
        return;
    }

    ncfr::Grid trial = currentResult_->grid;
    ncfr::Block& target = trial.at(pos.x, pos.y, pos.z);
    if (target.kind != ncfr::BlockKind::FuelCell) {
        hideFuelReplacementPanel();
        return;
    }
    target.type = fuelIndex;

    std::vector<int> fuelIndices = fuelIndicesInGrid(trial);
    if (!rebuildFuelCellPorts(trial, fuelIndices)) {
        QMessageBox::warning(this, QString::fromUtf8("警告"),
                             replacementFailureMessage(QString::fromUtf8("外壳空间不足，无法为替换后的燃料集合重建输入/输出燃料单元端口。")));
        return;
    }

    const ncfr::FuelSimulation sim = ncfr::simulateMixedFuel(trial);
    if (!ncfr::isSafeOperatingSimulation(trial, sim)) {
        QMessageBox::warning(this, QString::fromUtf8("警告"),
                             replacementFailureMessage(simulationFailureReason(trial, sim)));
        return;
    }

    currentResult_->grid = std::move(trial);
    currentResult_->request.fuelIndices = std::move(fuelIndices);
    currentResult_->minCoolingMargin = sim.minClusterMargin;
    currentResult_->usefulBlocks = countUsefulBlocks(currentResult_->grid);
    currentResult_->disconnectedFunctionalBlocks = sim.disconnectedFunctionalBlocks;
    currentResult_->functionalIrradiators = countFunctionalIrradiators(sim);
    currentResult_->irradiatorFlux = totalIrradiatorFlux(sim);

    const int currentLayer = layerSpin_ != nullptr ? layerSpin_->value() - 1 : pos.z;
    gridWidget_->setGrid(&currentResult_->grid);
    gridWidget_->setLayer(currentLayer);
    showFuelReplacementCandidates(pos.x, pos.y, pos.z, currentResult_->grid.index(pos.x, pos.y, pos.z));
    exportJsonButton_->setEnabled(true);
    statusBar()->showMessage(QString::fromUtf8("燃料已替换"));
}

void MainWindow::hideFuelReplacementPanel() {
    selectedFuelCell_.reset();
    if (fuelReplacementList_ != nullptr) {
        fuelReplacementList_->clear();
    }
    if (fuelReplacementTitle_ != nullptr) {
        fuelReplacementTitle_->clear();
    }
    if (fuelReplacementPanel_ != nullptr) {
        fuelReplacementPanel_->hide();
    }
}

void MainWindow::updateResultView() {
    if (!currentResult_.has_value()) {
        clearResultView();
        return;
    }

    hideFuelReplacementPanel();

    const ncfr::OptimizationResult& result = *currentResult_;
    layerSpin_->blockSignals(true);
    layerSpin_->setRange(1, result.grid.depth());
    layerSpin_->setValue(1);
    layerSpin_->setEnabled(true);
    layerSpin_->blockSignals(false);

    gridWidget_->setGrid(&result.grid);
    gridWidget_->setLayer(0);
}

void MainWindow::clearResultView() {
    currentResult_.reset();
    hideFuelReplacementPanel();
    exportJsonButton_->setEnabled(false);
    layerSpin_->setEnabled(false);
    layerSpin_->setRange(1, 1);
    gridWidget_->setGrid(nullptr);
}
