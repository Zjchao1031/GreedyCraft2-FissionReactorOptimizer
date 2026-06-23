#pragma once

#include "Optimizer.h"

#include <QMainWindow>

#include <atomic>
#include <memory>
#include <optional>
#include <vector>

class QCloseEvent;
class QComboBox;
class QDialog;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QString;
class QSpinBox;
class QThread;
class QVBoxLayout;
class ReactorGridWidget;

class MainWindow : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);

private:
    void closeEvent(QCloseEvent* event) override;

    QWidget* createInputPanel();
    QWidget* createGridPanel();
    QWidget* createLegendPanel();
    QWidget* createFuelReplacementPanel();

    struct FuelInputControls {
        QComboBox* fuelCellCountCombo = nullptr;
        QVBoxLayout* fuelRowsLayout = nullptr;
        std::vector<QComboBox*> fuelFamilyCombos;
        std::vector<QComboBox*> fuelCombos;
        QComboBox* irradiatorRecipeCombo = nullptr;
        QComboBox* moderatorTypeCombo = nullptr;
        QComboBox* reflectorTypeCombo = nullptr;
        QPushButton* generateButton = nullptr;
        int fixedFuelCellCount = 0;
    };

    QWidget* createFuelInputPanel(const QString& title, FuelInputControls& controls,
                                  QWidget* parent, const std::vector<int>& selectableFuelCounts,
                                  int fixedFuelCellCount);
    void generateLayout(const FuelInputControls& controls);
    void showGenerationDialog();
    void closeGenerationDialog();
    void requestCancelGeneration();
    void finishGenerationSuccess(ncfr::OptimizationResult result);
    void finishGenerationFailure(const QString& message);
    void finishGenerationCanceled();
    void exportResultAsJson();
    void showLayer(int zeroBasedLayer);
    void updateResultView();
    void clearResultView();
    void showFuelReplacementCandidates(int x, int y, int z, int index);
    void replaceSelectedFuel(QListWidgetItem* item);
    void hideFuelReplacementPanel();
    void rebuildFuelRows(FuelInputControls& controls);
    void updateFuelComboForRow(FuelInputControls& controls, int row);
    void setGenerationButtonsEnabled(bool enabled);
    ncfr::BuildRequest buildRequestFromUi(const FuelInputControls& controls) const;

    FuelInputControls normalInput_;
    FuelInputControls irradiatorInput_;
    QPushButton* exportJsonButton_ = nullptr;
    QSpinBox* layerSpin_ = nullptr;
    ReactorGridWidget* gridWidget_ = nullptr;
    QWidget* fuelReplacementPanel_ = nullptr;
    QLabel* fuelReplacementTitle_ = nullptr;
    QListWidget* fuelReplacementList_ = nullptr;
    QDialog* generationDialog_ = nullptr;
    QThread* generationThread_ = nullptr;
    std::shared_ptr<std::atomic_bool> generationCancelFlag_;

    std::optional<ncfr::OptimizationResult> currentResult_;
    std::optional<ncfr::Pos> selectedFuelCell_;
};
