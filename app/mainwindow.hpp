#pragma once

#include <QMainWindow>
#include <QFutureWatcher>

#include "sheetnest/cutting.hpp"
#include "sheetnest/dxf.hpp"
#include "sheetnest/nesting.hpp"

class QComboBox;
class QDoubleSpinBox;
class QPlainTextEdit;
class QLabel;
class QProgressBar;
class QSpinBox;
class QPushButton;

class NestView;

struct CalculationOutput {
    sheetnest::Result result;
    sheetnest::CuttingEstimate cutting;
    sheetnest::CuttingParameters technology;
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
    void exportDxf();
    void updateTechnologyPreview();
    void refreshInstances();
    void appendLog(const QString& text);
    void setBusy(bool busy);

    CalculationOutput performCalculation(
        std::vector<sheetnest::Instance> instances,
        sheetnest::Sheet sheet,
        sheetnest::Options options,
        sheetnest::CuttingParameters technology
    ) const;

    QString materialName(sheetnest::Material material) const;

    sheetnest::DxfDocument document_;
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
    QSpinBox* quantitySpin_{};
    QSpinBox* iterationsSpin_{};

    QCheckBox* rotation0_{};
    QCheckBox* rotation90_{};
    QCheckBox* rotation180_{};
    QCheckBox* rotation270_{};

    QLabel* fileLabel_{};
    QLabel* partCountLabel_{};
    QLabel* techLabel_{};
    QLabel* resultLabel_{};
    QProgressBar* progress_{};
    QPlainTextEdit* log_{};

    QPushButton* importButton_{};
    QPushButton* calculateButton_{};
    QPushButton* exportButton_{};

    QFutureWatcher<CalculationOutput>* watcher_{};
};
