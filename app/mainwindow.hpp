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
    void refreshAdaptiveRepairView();

    CalculationOutput performCalculation(
        std::vector<sheetnest::Instance> instances,
        sheetnest::Sheet sheet,
        sheetnest::Options options,
        sheetnest::CuttingParameters technology,
        sheetnest::ParallelNestingOptions parallelOptions
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
    QCheckBox* rotation270_{};

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
    sheetnest::ProductionValidationReport validation_{};

    std::shared_ptr<sheetnest::ParallelNestingController> nestingController_;
    QFutureWatcher<CalculationOutput>* watcher_{};
    QFutureWatcher<sheetnest::BenchmarkResult>* benchmarkWatcher_{};
};
