#pragma once

#include <QMainWindow>
#include <QFutureWatcher>

#include "sheetnest/benchmark.hpp"
#include "sheetnest/cutting.hpp"
#include "sheetnest/diagnostics.hpp"
#include "sheetnest/dxf.hpp"
#include "sheetnest/dxf_model.hpp"
#include "sheetnest/nesting.hpp"
#include "sheetnest/parallel_nesting.hpp"
#include "sheetnest/production_validation.hpp"

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QPlainTextEdit;
class QLabel;
class QProgressBar;
class QSpinBox;
class QPushButton;
class QTableWidget;
class QTabWidget;
class QTimer;

class NestView;

struct CalculationOutput {
    sheetnest::Result result;
    sheetnest::CuttingEstimate cutting;
    sheetnest::CuttingParameters technology;
    std::vector<sheetnest::InstanceDiagnostic> diagnostics;
    sheetnest::ProductionValidationReport validation;
};

Q_DECLARE_METATYPE(CalculationOutput)

class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private:
    void buildUi();
    void connectUi();
    void importDxf();
    void calculate();
    void repairErrors();
    void benchmark();
    void exportDxf();
    void populatePartTable();
    void populateDiagnostics();
    void populateBenchmark(const sheetnest::BenchmarkResult& benchmarkResult);
    void populateProductionValidation();
    void exportBenchmarkResults();
    void updateTechnologyPreview();
    void refreshInstances();
    void appendLog(const QString& text);
    void updateProgress(const sheetnest::NestingProgress& progress);
    void setBusy(bool busy);
    void stopCalculation(bool watchdogTriggered = false);
    void calculationWatchdogTick();
    void refreshAdaptiveRepairView();
    void resetAdaptiveRepairAnimation();
    void toggleAdaptiveRepairAnimation();
    void pauseAdaptiveRepairAnimation();
    void stepAdaptiveRepairAnimation(int direction);
    void advanceAdaptiveRepairAnimation();
    void updateAdaptiveRepairAnimationUi();
    void toggleLaserAnimation();
    void pauseLaserAnimation();
    void resetLaserAnimation();
    void advanceLaserAnimation();
    void updateLaserAnimationUi();
    void laserNextOperation();
    void laserPreviousOperation();
    void laserSelectOperation(int index);
    void populateLaserOperationSelector();

    CalculationOutput performCalculation(
        std::vector<sheetnest::Instance> instances,
        sheetnest::Sheet sheet,
        sheetnest::Options options,
        sheetnest::CuttingParameters technology,
        sheetnest::ParallelNestingOptions parallelOptions,
        std::shared_ptr<sheetnest::ParallelNestingController> controller
    ) const;

    sheetnest::BenchmarkResult performBenchmark(
        std::vector<sheetnest::Instance> instances,
        sheetnest::Sheet sheet,
        sheetnest::Options options
    ) const;

    QString materialName(sheetnest::Material material) const;

    sheetnest::DxfDocument document_;
    std::vector<sheetnest::Part> parts_;
    std::vector<sheetnest::Instance> instances_;
    sheetnest::Result result_;
    sheetnest::Sheet sheet_;
    sheetnest::Options options_;
    sheetnest::CuttingParameters technology_;
    QString currentFile_;

    NestView* view_{};
    QComboBox* materialCombo_{};
    QDoubleSpinBox* thicknessSpin_{};
    QDoubleSpinBox* sheetWidthSpin_{};
    QDoubleSpinBox* sheetHeightSpin_{};
    QDoubleSpinBox* marginSpin_{};
    QDoubleSpinBox* gapSpin_{};
    QSpinBox* iterationsSpin_{};
    QSpinBox* workersSpin_{};
    QSpinBox* timeBudgetSpin_{};

    QCheckBox* rotation0_{};
    QCheckBox* rotation90_{};
    QCheckBox* rotation180_{};
    QCheckBox* rotation270_{};\n    QCheckBox* cuttingRouteCheck_{};
    QPushButton* laserPlayButton_{};
    QPushButton* laserPauseButton_{};
    QPushButton* laserResetButton_{};
    QComboBox* laserSpeedCombo_{};
    QComboBox* laserOperationCombo_{};
    QPushButton* laserPrevButton_{};
    QPushButton* laserNextButton_{};
    QLabel* laserStageLabel_{};

    QLabel* fileLabel_{};
    QLabel* partCountLabel_{};
    QLabel* techLabel_{};
    QLabel* resultLabel_{};
    QLabel* progressDetails_{};
    QProgressBar* progress_{};
    QPlainTextEdit* log_{};
    QTableWidget* partTable_{};
    QTableWidget* diagnosticsTable_{};
    QTableWidget* benchmarkTable_{};
    QTableWidget* validatorTable_{};

    QComboBox* repairRoundCombo_{};
    QCheckBox* repairConflictLayer_{};
    QCheckBox* repairExtractedLayer_{};
    QCheckBox* repairMovedLayer_{};
    QCheckBox* repairStationaryLayer_{};
    QPushButton* repairPlayButton_{};
    QPushButton* repairPauseButton_{};
    QPushButton* repairPrevButton_{};
    QPushButton* repairNextButton_{};
    QComboBox* repairSpeedCombo_{};
    QLabel* repairStageLabel_{};

    QPushButton* importButton_{};
    QPushButton* calculateButton_{};
    QPushButton* repairButton_{};
    QPushButton* benchmarkButton_{};
    QPushButton* benchmarkExportButton_{};
    QPushButton* stopButton_{};
    QPushButton* exportButton_{};

    sheetnest::BenchmarkResult lastBenchmarkResult_{};
    bool hasBenchmarkResult_{false};
    bool repairRequested_{false};
    bool repairAnimationPlaying_{false};
    bool repairAnimationSession_{false};
    int repairAnimationFrame_{-1};
    sheetnest::ProductionValidationReport validation_{};

    QTimer* repairAnimationTimer_{};
    QTimer* calculationWatchdog_{};
    QTimer* laserAnimationTimer_{};
    bool laserAnimationPlaying_{false};
    double laserAnimationProgress_{1.0};
    int laserAnimationOperation_{-1};
    qint64 calculationStartedMs_{0};
    qint64 lastProgressMs_{0};
    bool watchdogTriggered_{false};
    bool userCancelRequested_{false};

    std::shared_ptr<sheetnest::ParallelNestingController> nestingController_;
    QFutureWatcher<CalculationOutput>* watcher_{};
    QFutureWatcher<sheetnest::BenchmarkResult>* benchmarkWatcher_{};
};
