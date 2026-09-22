#include "mainwindow.hpp"
#include "nestview.hpp"
#include "sheetnest/cutting_path.hpp"
#include "sheetnest/dxf_export.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QCheckBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QSaveFile>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QPointer>
#include <QSplitter>
#include <QSpinBox>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QTableWidget>
#include <QHeaderView>
#include <QTabWidget>
#include <QTimer>
#include <QSignalBlocker>
#include <QSlider>
#include <QAbstractItemView>
#include <QtConcurrent>

#include <algorithm>
#include <cmath>
#include <unordered_map>

using namespace sheetnest;

namespace {

QString severityName(DxfSeverity severity) {
    switch (severity) {
    case DxfSeverity::Info: return "INFO";
    case DxfSeverity::Warning: return "WARNING";
    case DxfSeverity::Error: return "ERROR";
    }
    return "UNKNOWN";
}

QVector<Polygon> placementPolygons(
    const Instance& instance,
    const Placement& placement
) {
    QVector<Polygon> result;

    result.push_back(
        translate(
            rotate(instance.part.outer, placement.rotation),
            placement.x,
            placement.y
        )
    );

    for (const auto& hole : instance.part.holes) {
        result.push_back(
            translate(
                rotate(hole, placement.rotation),
                placement.x,
                placement.y
            )
        );
    }

    return result;
}

CuttingPath planWholeResultRoute(
    const Result& result,
    const std::vector<Instance>& instances,
    const CuttingParameters& technology
) {
    std::unordered_map<std::string, const Instance*> byId;
    byId.reserve(instances.size());
    for (const auto& instance : instances) byId.emplace(instance.id, &instance);

    std::vector<CuttingContour> contours;
    for (std::size_t sheetIndex = 0; sheetIndex < result.sheets.size(); ++sheetIndex) {
        for (const auto& placement : result.sheets[sheetIndex]) {
            const auto it = byId.find(placement.id);
            if (it == byId.end()) continue;

            contours.push_back({
                sheetIndex, placement.id, 0, false,
                translate(rotate(it->second->part.outer, placement.rotation),
                          placement.x, placement.y)
            });

            for (std::size_t holeIndex = 0;
                 holeIndex < it->second->part.holes.size();
                 ++holeIndex) {
                contours.push_back({
                    sheetIndex, placement.id, holeIndex, true,
                    translate(
                        rotate(it->second->part.holes[holeIndex], placement.rotation),
                        placement.x, placement.y
                    )
                });
            }
        }
    }

    PathOptions pathOptions;
    pathOptions.rapidSpeedMMin = 120.0;
    pathOptions.pierceSeconds = 0.25;
    pathOptions.innerContoursFirst = true;
    return planCuttingRoute(contours, technology, pathOptions);
}

CuttingEstimate estimateWholeResult(
    const Result& result,
    const std::vector<Instance>& instances,
    const CuttingParameters& technology
) {
    const auto route = planWholeResultRoute(result, instances, technology);
    PathOptions options;
    options.rapidSpeedMMin = 120.0;
    options.pierceSeconds = 0.25;
    return estimateCuttingPath(route, technology, options);
}

QDoubleSpinBox* makeDouble(
    double min,
    double max,
    double value,
    double step
) {
    auto* spin = new QDoubleSpinBox;
    spin->setRange(min, max);
    spin->setValue(value);
    spin->setSingleStep(step);
    spin->setDecimals(2);
    return spin;
}

} // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      watcher_(new QFutureWatcher<CalculationOutput>(this)),
      benchmarkWatcher_(new QFutureWatcher<sheetnest::BenchmarkResult>(this))
{
    buildUi();
    connectUi();

    setWindowTitle("SheetNest 2.0 — раскрой металла");
    resize(1500, 900);
    statusBar()->showMessage("Готово");
    updateTechnologyPreview();
}

MainWindow::~MainWindow() = default;

void MainWindow::buildUi() {
    auto* splitter = new QSplitter(Qt::Horizontal, this);

    auto* controlPanel = new QWidget;
    auto* controlLayout = new QVBoxLayout(controlPanel);
    controlLayout->setContentsMargins(14, 14, 14, 14);
    controlLayout->setSpacing(10);

    auto* title = new QLabel("SHEETNEST 2.0");
    title->setObjectName("title");
    controlLayout->addWidget(title);

    importButton_ = new QPushButton("Загрузить DXF");
    calculateButton_ = new QPushButton("Рассчитать раскрой");
    calculateButton_->setEnabled(false);

    repairButton_ = new QPushButton("Исправить ошибки");
    repairButton_->setEnabled(false);

    benchmarkButton_ = new QPushButton("Benchmark до / после оптимизации");
    benchmarkButton_->setEnabled(false);

    exportButton_ = new QPushButton("Экспорт раскладки DXF");
    exportButton_->setEnabled(false);

    fileLabel_ = new QLabel("Файл не загружен");
    fileLabel_->setWordWrap(true);
    controlLayout->addWidget(importButton_);
    controlLayout->addWidget(fileLabel_);

    auto* sheetGroup = new QGroupBox("Лист");
    auto* sheetForm = new QFormLayout(sheetGroup);

    sheetWidthSpin_ = makeDouble(100.0, 20000.0, 1500.0, 10.0);
    sheetHeightSpin_ = makeDouble(100.0, 40000.0, 3000.0, 10.0);
    marginSpin_ = makeDouble(0.0, 500.0, 5.0, 1.0);
    gapSpin_ = makeDouble(0.0, 50.0, 2.0, 0.5);

    sheetForm->addRow("Ширина, мм", sheetWidthSpin_);
    sheetForm->addRow("Высота, мм", sheetHeightSpin_);
    sheetForm->addRow("Припуск от края, мм", marginSpin_);
    sheetForm->addRow("Зазор, мм", gapSpin_);
    controlLayout->addWidget(sheetGroup);

    auto* partsGroup = new QGroupBox("Детали и количество");
    auto* partsLayout = new QVBoxLayout(partsGroup);

    partTable_ = new QTableWidget(0, 4);
    partTable_->setHorizontalHeaderLabels({
        "Деталь", "Слой", "Source ID", "Количество"
    });
    partTable_->horizontalHeader()->setStretchLastSection(true);
    partTable_->horizontalHeader()->setSectionResizeMode(
        0, QHeaderView::Stretch
    );
    partTable_->horizontalHeader()->setSectionResizeMode(
        1, QHeaderView::ResizeToContents
    );
    partTable_->horizontalHeader()->setSectionResizeMode(
        2, QHeaderView::Stretch
    );
    partTable_->setSelectionBehavior(
        QAbstractItemView::SelectRows
    );
    partTable_->setEditTriggers(
        QAbstractItemView::DoubleClicked |
        QAbstractItemView::EditKeyPressed |
        QAbstractItemView::SelectedClicked
    );
    partTable_->setMinimumHeight(150);
    partsLayout->addWidget(partTable_);

    auto* settingsForm = new QFormLayout;

    iterationsSpin_ = new QSpinBox;
    iterationsSpin_->setRange(1, 128);
    iterationsSpin_->setValue(24);
    settingsForm->addRow("Итерации оптимизации", iterationsSpin_);

    workersSpin_ = new QSpinBox;
    workersSpin_->setRange(0, 64);
    workersSpin_->setValue(0);
    workersSpin_->setSpecialValueText("Авто");
    settingsForm->addRow("Параллельные workers", workersSpin_);

    timeBudgetSpin_ = new QSpinBox;
    timeBudgetSpin_->setRange(5, 3600);
    timeBudgetSpin_->setValue(120);
    timeBudgetSpin_->setSuffix(" с");
    settingsForm->addRow("Лимит расчёта", timeBudgetSpin_);

    auto* rotationWidget = new QWidget;
    auto* rotationLayout = new QGridLayout(rotationWidget);
    rotationLayout->setContentsMargins(0, 0, 0, 0);

    rotation0_ = new QCheckBox("0°");
    rotation90_ = new QCheckBox("90°");
    rotation180_ = new QCheckBox("180°");
    rotation270_ = new QCheckBox("270°");
    rotation0_->setChecked(true);
    rotation90_->setChecked(true);
    rotation180_->setChecked(true);
    rotation270_->setChecked(true);

    rotationLayout->addWidget(rotation0_, 0, 0);
    rotationLayout->addWidget(rotation90_, 0, 1);
    rotationLayout->addWidget(rotation180_, 1, 0);
    rotationLayout->addWidget(rotation270_, 1, 1);

    settingsForm->addRow("Повороты", rotationWidget);
    partCountLabel_ = new QLabel("Деталей: 0");
    settingsForm->addRow(partCountLabel_);

    partsLayout->addLayout(settingsForm);
    controlLayout->addWidget(partsGroup);

    auto* technologyGroup = new QGroupBox("Лазер 3 кВт Bodor");
    auto* technologyForm = new QFormLayout(technologyGroup);

    materialCombo_ = new QComboBox;
    materialCombo_->addItem("Конструкционная сталь");
    materialCombo_->addItem("Нержавеющая сталь");
    materialCombo_->addItem("Алюминий");
    materialCombo_->addItem("Латунь");

    thicknessSpin_ = makeDouble(0.5, 30.0, 3.0, 0.5);
    techLabel_ = new QLabel;
    techLabel_->setWordWrap(true);

    technologyForm->addRow("Материал", materialCombo_);
    technologyForm->addRow("Толщина, мм", thicknessSpin_);
    technologyForm->addRow("Технология", techLabel_);

    controlLayout->addWidget(technologyGroup);

    resultLabel_ = new QLabel("Расчёт ещё не выполнялся");
    resultLabel_->setWordWrap(true);
    controlLayout->addWidget(resultLabel_);

    progress_ = new QProgressBar;
    progress_->setRange(0, 1);
    progress_->setValue(0);
    controlLayout->addWidget(progress_);

    progressDetails_ = new QLabel("Ожидание расчёта");
    progressDetails_->setWordWrap(true);
    controlLayout->addWidget(progressDetails_);

    controlLayout->addWidget(calculateButton_);
    controlLayout->addWidget(repairButton_);

    stopButton_ = new QPushButton("Остановить расчёт");
    stopButton_->setEnabled(false);
    controlLayout->addWidget(stopButton_);

    benchmarkButton_ = new QPushButton("Benchmark до / после оптимизации");
    controlLayout->addWidget(benchmarkButton_);

    benchmarkExportButton_ = new QPushButton("Экспорт результатов Benchmark");
    benchmarkExportButton_->setEnabled(false);
    controlLayout->addWidget(benchmarkExportButton_);

    cuttingRouteCheck_ = new QCheckBox("Показать маршрут лазера");
    cuttingRouteCheck_->setChecked(true);
    cuttingRouteCheck_->setToolTip(
        "Пробивка, внутренние и внешние контуры, rapid-переходы и направление движения"
    );
    controlLayout->addWidget(cuttingRouteCheck_);

    auto* laserControls = new QGroupBox("Анимация лазерной головки");
    auto* laserLayout = new QGridLayout(laserControls);

    laserPlayButton_ = new QPushButton("▶ Запуск");
    laserPauseButton_ = new QPushButton("⏸ Пауза");
    laserResetButton_ = new QPushButton("↺ В начало");
    laserPrevButton_ = new QPushButton("◀ Предыдущая");
    laserNextButton_ = new QPushButton("Следующая ▶");
    laserContourStartButton_ = new QPushButton("⏮ Начало контура");
    laserContourMiddleButton_ = new QPushButton("⏺ 50%");
    laserContourEndButton_ = new QPushButton("Конец ⏭");
    laserContourStepBackButton_ = new QPushButton("− Шаг");
    laserContourStepForwardButton_ = new QPushButton("Шаг +");

    laserOperationCombo_ = new QComboBox;
    laserOperationCombo_->setMinimumWidth(260);
    laserOperationCombo_->setToolTip(
        "Выберите деталь или контур для перехода к нему"
    );

    laserContourProgressSlider_ = new QSlider(Qt::Horizontal);
    laserContourProgressSlider_->setRange(0, 100);
    laserContourProgressSlider_->setValue(0);
    laserContourProgressSlider_->setSingleStep(5);
    laserContourProgressSlider_->setPageStep(10);
    laserContourProgressSlider_->setToolTip(
        "Положение внутри выбранного контура: 0% — начало, 50% — середина, 100% — конец"
    );

    laserSpeedCombo_ = new QComboBox;
    laserSpeedCombo_->addItem("0.5×", 0.5);
    laserSpeedCombo_->addItem("1×", 1.0);
    laserSpeedCombo_->addItem("2×", 2.0);
    laserSpeedCombo_->addItem("4×", 4.0);
    laserSpeedCombo_->setCurrentIndex(1);

    laserStageLabel_ = new QLabel("Готово • маршрут завершён");
    laserStageLabel_->setWordWrap(true);

    laserAnimationTimer_ = new QTimer(this);
    laserAnimationTimer_->setInterval(100);

    laserLayout->addWidget(laserPlayButton_, 0, 0);
    laserLayout->addWidget(laserPauseButton_, 0, 1);
    laserLayout->addWidget(laserResetButton_, 0, 2);
    laserLayout->addWidget(laserPrevButton_, 0, 3);
    laserLayout->addWidget(laserNextButton_, 0, 4);
    laserLayout->addWidget(new QLabel("Операция:"), 1, 0);
    laserLayout->addWidget(laserOperationCombo_, 1, 1, 1, 4);
    laserLayout->addWidget(new QLabel("Контур:"), 2, 0);
    laserLayout->addWidget(laserContourStartButton_, 2, 1);
    laserLayout->addWidget(laserContourMiddleButton_, 2, 2);
    laserLayout->addWidget(laserContourEndButton_, 2, 3);
    laserLayout->addWidget(laserContourStepBackButton_, 3, 0);
    laserLayout->addWidget(laserContourProgressSlider_, 3, 1, 1, 3);
    laserLayout->addWidget(laserContourStepForwardButton_, 3, 4);
    laserLayout->addWidget(new QLabel("Скорость:"), 4, 0);
    laserLayout->addWidget(laserSpeedCombo_, 4, 1);
    laserLayout->addWidget(laserStageLabel_, 4, 2, 1, 3);

    laserPauseButton_->setEnabled(false);
    laserResetButton_->setEnabled(false);
    laserPrevButton_->setEnabled(false);
    laserNextButton_->setEnabled(false);
    laserContourStartButton_->setEnabled(false);
    laserContourMiddleButton_->setEnabled(false);
    laserContourEndButton_->setEnabled(false);
    laserContourStepBackButton_->setEnabled(false);
    laserContourStepForwardButton_->setEnabled(false);
    laserContourProgressSlider_->setEnabled(false);
    laserOperationCombo_->setEnabled(false);
    laserSpeedCombo_->setEnabled(false);

    controlLayout->addWidget(laserControls);

    controlLayout->addWidget(exportButton_);

    log_ = new QPlainTextEdit;
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(2000);
    log_->setMinimumHeight(160);
    controlLayout->addWidget(log_, 1);

    view_ = new NestView;
    view_->setCuttingRouteVisible(true);

    auto* repairViewPanel = new QWidget;
    auto* repairViewLayout = new QVBoxLayout(repairViewPanel);
    repairViewLayout->setContentsMargins(0, 0, 0, 0);
    repairViewLayout->setSpacing(4);

    auto* repairControls = new QGroupBox("Adaptive Repair");
    auto* repairControlsLayout = new QGridLayout(repairControls);
    repairControlsLayout->setContentsMargins(8, 6, 8, 6);

    repairRoundCombo_ = new QComboBox;
    repairRoundCombo_->addItem("Итоговая раскладка", -1);
    repairRoundCombo_->setEnabled(false);

    repairConflictLayer_ = new QCheckBox("Конфликты");
    repairExtractedLayer_ = new QCheckBox("Извлечённые");
    repairMovedLayer_ = new QCheckBox("Новые позиции");
    repairStationaryLayer_ = new QCheckBox("Неподвижные");
    repairConflictLayer_->setChecked(true);
    repairExtractedLayer_->setChecked(true);
    repairMovedLayer_->setChecked(true);
    repairStationaryLayer_->setChecked(true);

    repairPlayButton_ = new QPushButton("▶");
    repairPauseButton_ = new QPushButton("⏸");
    repairPrevButton_ = new QPushButton("◀");
    repairNextButton_ = new QPushButton("▶|");
    repairSpeedCombo_ = new QComboBox;
    repairSpeedCombo_->addItem("0.5×", 0.5);
    repairSpeedCombo_->addItem("1×", 1.0);
    repairSpeedCombo_->addItem("2×", 2.0);
    repairSpeedCombo_->setCurrentIndex(1);

    repairStageLabel_ = new QLabel("Анимация: готово");
    repairStageLabel_->setWordWrap(true);

    repairAnimationTimer_ = new QTimer(this);
    repairAnimationTimer_->setInterval(900);

    calculationWatchdog_ = new QTimer(this);
    calculationWatchdog_->setInterval(1000);

    repairControlsLayout->addWidget(
        new QLabel("История:"), 0, 0
    );
    repairControlsLayout->addWidget(
        repairRoundCombo_, 0, 1, 1, 3
    );
    repairControlsLayout->addWidget(
        repairConflictLayer_, 1, 0
    );
    repairControlsLayout->addWidget(
        repairExtractedLayer_, 1, 1
    );
    repairControlsLayout->addWidget(
        repairMovedLayer_, 1, 2
    );
    repairControlsLayout->addWidget(
        repairStationaryLayer_, 1, 3
    );

    repairControlsLayout->addWidget(
        repairPlayButton_, 2, 0
    );
    repairControlsLayout->addWidget(
        repairPauseButton_, 2, 1
    );
    repairControlsLayout->addWidget(
        repairPrevButton_, 2, 2
    );
    repairControlsLayout->addWidget(
        repairNextButton_, 2, 3
    );
    repairControlsLayout->addWidget(
        new QLabel("Скорость:"), 3, 0
    );
    repairControlsLayout->addWidget(
        repairSpeedCombo_, 3, 1
    );
    repairControlsLayout->addWidget(
        repairStageLabel_, 3, 2, 1, 2
    );

    repairPlayButton_->setEnabled(false);
    repairPauseButton_->setEnabled(false);
    repairPrevButton_->setEnabled(false);
    repairNextButton_->setEnabled(false);
    repairSpeedCombo_->setEnabled(false);

    repairViewLayout->addWidget(repairControls);
    repairViewLayout->addWidget(view_, 1);
    diagnosticsTable_->setHorizontalHeaderLabels({
        "instanceId", "unitId", "Source ID", "Слой", "Этап", "Сообщение"
    });
    diagnosticsTable_->horizontalHeader()->setStretchLastSection(true);
    diagnosticsTable_->setSelectionBehavior(
        QAbstractItemView::SelectRows
    );
    diagnosticsTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);

    benchmarkTable_ = new QTableWidget(0, 15);
    benchmarkTable_->setHorizontalHeaderLabels({
        "Режим", "Время, мс", "Листов", "Размещено", "Пропущено",
        "Использование", "Кандидаты", "Collision checks", "NFP checks",
        "Refill moves", "Exchange attempts", "Sheets eliminated",
        "Optimizer passes", "NFP timeouts", "NFP fallbacks"
    });
    benchmarkTable_->horizontalHeader()->setStretchLastSection(true);
    benchmarkTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    benchmarkTable_->setSelectionBehavior(
        QAbstractItemView::SelectRows
    );

    validatorTable_ = new QTableWidget(0, 7);
    validatorTable_->setHorizontalHeaderLabels({
        "Тип", "Лист", "instanceId", "Связанный ID",
        "Измерено, мм", "Требуется, мм", "Сообщение"
    });
    validatorTable_->horizontalHeader()->setStretchLastSection(true);
    validatorTable_->setSelectionBehavior(
        QAbstractItemView::SelectRows
    );
    validatorTable_->setEditTriggers(QAbstractItemView::NoEditTriggers);

    auto* rightTabs = new QTabWidget;
    rightTabs->addTab(repairViewPanel, "Раскладка");
    rightTabs->addTab(diagnosticsTable_, "Диагностика");
    rightTabs->addTab(validatorTable_, "Production Validator");
    rightTabs->addTab(benchmarkTable_, "Benchmark");

    splitter->addWidget(controlPanel);
    splitter->addWidget(rightTabs);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({420, 1080});

    setCentralWidget(splitter);

    setStyleSheet(R"QSS(
        QMainWindow {
            background: #0b0f14;
            color: #e5e7eb;
        }
        QWidget {
            background: #111820;
            color: #e5e7eb;
            font-size: 10pt;
        }
        #title {
            font-size: 20pt;
            font-weight: 700;
            color: #f8fafc;
            padding: 4px 0 10px 0;
        }
        QGroupBox {
            border: 1px solid #334155;
            border-radius: 8px;
            margin-top: 8px;
            padding-top: 8px;
            background: #151d26;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
            color: #94a3b8;
        }
        QPushButton {
            background: #1d4ed8;
            border: 0;
            border-radius: 6px;
            padding: 9px;
            font-weight: 600;
        }
        QPushButton:hover {
            background: #2563eb;
        }
        QPushButton:disabled {
            background: #334155;
            color: #94a3b8;
        }
        QDoubleSpinBox, QSpinBox, QComboBox, QPlainTextEdit {
            background: #0f1720;
            border: 1px solid #334155;
            border-radius: 5px;
            padding: 5px;
        }
        QLabel {
            color: #cbd5e1;
        }
        QProgressBar {
            border: 1px solid #334155;
            border-radius: 5px;
            background: #0f1720;
            min-height: 8px;
        }
        QProgressBar::chunk {
            background: #38bdf8;
            border-radius: 4px;
        }
        QCheckBox {
            spacing: 5px;
        }
    )QSS");
}

void MainWindow::connectUi() {
    connect(importButton_, &QPushButton::clicked, this, [this] {
        importDxf();
    });

    connect(stopButton_, &QPushButton::clicked, this, [this] {
        stopCalculation(false);
    });

    connect(calculationWatchdog_, &QTimer::timeout, this, [this] {
        calculationWatchdogTick();
    });

    connect(calculateButton_, &QPushButton::clicked, this, [this] {
        repairRequested_ = false;
        calculate();
    });

    connect(repairButton_, &QPushButton::clicked, this, [this] {
        repairErrors();
    });

    const auto rerenderAdaptive = [this] {
        refreshAdaptiveRepairView();
    };

    connect(
        repairRoundCombo_,
        qOverload<int>(&QComboBox::currentIndexChanged),
        this,
        [rerenderAdaptive](int) {
            rerenderAdaptive();
        }
    );
    connect(
        repairConflictLayer_,
        &QCheckBox::toggled,
        this,
        [rerenderAdaptive](bool) {
            rerenderAdaptive();
        }
    );
    connect(
        repairExtractedLayer_,
        &QCheckBox::toggled,
        this,
        [rerenderAdaptive](bool) {
            rerenderAdaptive();
        }
    );
    connect(
        repairMovedLayer_,
        &QCheckBox::toggled,
        this,
        [rerenderAdaptive](bool) {
            rerenderAdaptive();
        }
    );
    connect(
        repairStationaryLayer_,
        &QCheckBox::toggled,
        this,
        [rerenderAdaptive](bool) {
            rerenderAdaptive();
        }
    );

    connect(
        repairPlayButton_,
        &QPushButton::clicked,
        this,
        [this] {
            toggleAdaptiveRepairAnimation();
        }
    );
    connect(
        repairPauseButton_,
        &QPushButton::clicked,
        this,
        [this] {
            pauseAdaptiveRepairAnimation();
        }
    );
    connect(
        repairPrevButton_,
        &QPushButton::clicked,
        this,
        [this] {
            stepAdaptiveRepairAnimation(-1);
        }
    );
    connect(
        repairNextButton_,
        &QPushButton::clicked,
        this,
        [this] {
            stepAdaptiveRepairAnimation(1);
        }
    );
    connect(
        repairSpeedCombo_,
        qOverload<int>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            updateAdaptiveRepairAnimationUi();
        }
    );
    connect(
        repairAnimationTimer_,
        &QTimer::timeout,
        this,
        [this] {
            advanceAdaptiveRepairAnimation();
        }
    );

    connect(
        cuttingRouteCheck_,
        &QCheckBox::toggled,
        this,
        [this](bool visible) {
            view_->setCuttingRouteVisible(visible);
            if (!visible) {
                pauseLaserAnimation();
            }
            updateLaserAnimationUi();
            refreshAdaptiveRepairView();
        }
    );
    connect(
        laserPlayButton_,
        &QPushButton::clicked,
        this,
        [this] {
            toggleLaserAnimation();
        }
    );
    connect(
        laserPauseButton_,
        &QPushButton::clicked,
        this,
        [this] {
            pauseLaserAnimation();
        }
    );
    connect(
        laserResetButton_,
        &QPushButton::clicked,
        this,
        [this] {
            resetLaserAnimation();
        }
    );

    connect(
        laserPrevButton_,
        &QPushButton::clicked,
        this,
        [this] {
            laserPreviousOperation();
        }
    );
    connect(
        laserNextButton_,
        &QPushButton::clicked,
        this,
        [this] {
            laserNextOperation();
        }
    );
    connect(
        laserOperationCombo_,
        qOverload<int>(&QComboBox::currentIndexChanged),
        this,
        [this](int index) {
            if (index >= 0) {
                laserSelectOperation(index);
            }
        }
    );
    connect(
        laserSpeedCombo_,
        qOverload<int>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            updateLaserAnimationUi();
        }
    );
    connect(
        laserAnimationTimer_,
        &QTimer::timeout,
        this,
        [this] {
            advanceLaserAnimation();
        }
    );

    connect(exportButton_, &QPushButton::clicked, this, [this] {
        exportDxf();
    });

    connect(partTable_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) {
        refreshInstances();
    });

    connect(benchmarkButton_, &QPushButton::clicked, this, [this] {
        benchmark();
    });

    connect(benchmarkExportButton_, &QPushButton::clicked, this, [this] {
        exportBenchmarkResults();
    });

    connect(materialCombo_,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this,
            [this](int) {
        updateTechnologyPreview();
    });

    connect(thicknessSpin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            [this](double value) {
        Q_UNUSED(value);
        updateTechnologyPreview();
    });

    connect(watcher_, &QFutureWatcher<CalculationOutput>::finished,
            this,
            [this] {
        try {
            const auto output = watcher_->result();
            result_ = output.result;
            technology_ = output.technology;
            validation_ = output.validation;
            cuttingRoute_ = output.cuttingRoute;
            resetAdaptiveRepairAnimation();
            resetLaserAnimation();

            {
                const QSignalBlocker blocker(
                    repairRoundCombo_
                );
                repairRoundCombo_->clear();
                repairRoundCombo_->addItem(
                    "Итоговая раскладка",
                    -1
                );
                for (const auto& round :
                     validation_.adaptiveHistory) {
                    repairRoundCombo_->addItem(
                        QString("Раунд %1")
                            .arg(static_cast<qulonglong>(
                                round.roundIndex)),
                        static_cast<int>(round.roundIndex)
                    );
                }
                const bool hasHistory =
                    !validation_.adaptiveHistory.empty();
                repairRoundCombo_->setEnabled(hasHistory);
                if (repairConflictLayer_) {
                    repairConflictLayer_->setEnabled(hasHistory);
                }
                if (repairExtractedLayer_) {
                    repairExtractedLayer_->setEnabled(hasHistory);
                }
                if (repairMovedLayer_) {
                    repairMovedLayer_->setEnabled(hasHistory);
                }
                if (repairStationaryLayer_) {
                    repairStationaryLayer_->setEnabled(hasHistory);
                }
                if (repairPlayButton_) {
                    repairPlayButton_->setEnabled(hasHistory);
                }
                if (repairSpeedCombo_) {
                    repairSpeedCombo_->setEnabled(hasHistory);
                }
            }

            updateAdaptiveRepairAnimationUi();
            populateDiagnostics();
            populateProductionValidation();

            refreshAdaptiveRepairView();

            const auto minutes = output.cutting.totalMinutes;
            const int hours = static_cast<int>(minutes / 60.0);
            const int mins = static_cast<int>(
                std::floor(minutes)
            ) % 60;

            resultLabel_->setText(
                QString("Листов: %1\n"
                        "Размещено: %2\n"
                        "Не размещено: %3\n"
                        "Использование: %4%\n"
                        "Production Validator: %5\n"
                        "Длина реза: %6 м\n"
                        "Пробивок: %7\n"
                        "Время лазерной резки: %8 ч %9 мин")
                    .arg(static_cast<int>(result_.sheets.size()))
                    .arg(static_cast<int>(
                        instances_.size() - result_.unplaced.size()))
                    .arg(static_cast<int>(result_.unplaced.size()))
                    .arg(result_.utilization * 100.0, 0, 'f', 1)
                    .arg(validation_.valid ? "OK" : "ОШИБКА")
                    .arg(output.cutting.contourLengthMm / 1000.0, 0, 'f', 2)
                    .arg(output.cutting.pierces)
                    .arg(hours)
                    .arg(mins)
            );

            if (watchdogTriggered_) {
                appendLog("Расчёт завершён после срабатывания GUI Watchdog.");
            } else if (userCancelRequested_) {
                appendLog("Расчёт завершён после отмены пользователем.");
            }

                        appendLog(
                QString("Расчёт завершён: %1 листов, использование %2%.")
                    .arg(static_cast<int>(result_.sheets.size()))
                    .arg(result_.utilization * 100.0, 0, 'f', 1)
            );
            appendLog(
                QString("Bodor 3 кВт: %1 м/мин, газ %2, время %3 мин.")
                    .arg(technology_.speedMMin, 0, 'f', 2)
                    .arg(QString::fromStdString(technology_.assistGas))
                    .arg(output.cutting.totalMinutes, 0, 'f', 1)
            );

            if (!result_.unplaced.empty()) {
                appendLog(
                    QString("Не размещено: %1")
                        .arg(static_cast<int>(result_.unplaced.size()))
                );
            }

            appendLog(
                QString("NFP telemetry: таймауты=%1, complexity fallback=%2.")
                    .arg(static_cast<qulonglong>(result_.stats.nfpTimeouts))
                    .arg(static_cast<qulonglong>(result_.stats.nfpComplexityFallbacks))
            );

            appendLog(
                QString("Production Validator: %1; collision=%2, gap=%3, margin=%4, duplicate ID=%5, missing ID=%6.")
                    .arg(validation_.valid ? "OK" : "ОШИБКА")
                    .arg(static_cast<qulonglong>(validation_.collisionCount))
                    .arg(static_cast<qulonglong>(validation_.gapViolationCount))
                    .arg(static_cast<qulonglong>(validation_.marginViolationCount))
                    .arg(static_cast<qulonglong>(validation_.duplicateIdCount))
                    .arg(static_cast<qulonglong>(validation_.missingIdCount))
            );

            if (validation_.repairAttempts > 0 ||
                validation_.adaptiveRepairRounds > 0) {
                appendLog(
                    QString("Auto Repair: %1; adaptive rounds=%2, group=%3, "
                            "global attempts=%4, время=%5 мс.")
                        .arg(validation_.repaired ? "успешно" : "не удалось")
                        .arg(static_cast<qulonglong>(
                            validation_.adaptiveRepairRounds))
                        .arg(static_cast<qulonglong>(
                            validation_.adaptiveRepairGroupSize))
                        .arg(static_cast<qulonglong>(
                            validation_.repairAttempts))
                        .arg(static_cast<qulonglong>(
                            validation_.repairElapsedMs))
                );
            }

            exportButton_->setEnabled(
                !result_.sheets.empty() &&
                validation_.valid
            );
        } catch (const std::exception& error) {
            QMessageBox::critical(
                this,
                "Ошибка расчёта",
                QString::fromUtf8(error.what())
            );
            appendLog(
                QString("Ошибка расчёта: %1")
                    .arg(QString::fromUtf8(error.what()))
            );
        }

        if (calculationWatchdog_) calculationWatchdog_->stop();
        repairRequested_ = false;
        nestingController_.reset();
        setBusy(false);
    });

    connect(
        benchmarkWatcher_,
        &QFutureWatcher<sheetnest::BenchmarkResult>::finished,
        this,
        [this] {
            try {
                const auto benchmarkResult = benchmarkWatcher_->result();
                lastBenchmarkResult_ = benchmarkResult;
                hasBenchmarkResult_ = true;
                benchmarkExportButton_->setEnabled(true);
                populateBenchmark(benchmarkResult);
                appendLog(
                    QString("Benchmark: базовый %1 мс / %2 листов / %3 кандидатов / %4 NFP / refill %5 / exchange %6 / eliminated %7 / passes %8; оптимизированный %9 мс / %10 листов / %11 кандидатов / %12 NFP / refill %13 / exchange %14 / eliminated %15 / passes %16.")
                        .arg(benchmarkResult.baseline.milliseconds, 0, 'f', 1)
                        .arg(static_cast<int>(benchmarkResult.baseline.sheets))
                        .arg(static_cast<qulonglong>(benchmarkResult.baseline.candidateChecks))
                        .arg(static_cast<qulonglong>(benchmarkResult.baseline.nfpChecks))
                        .arg(static_cast<qulonglong>(benchmarkResult.baseline.refillMoves))
                        .arg(static_cast<qulonglong>(benchmarkResult.baseline.exchangeAttempts))
                        .arg(static_cast<qulonglong>(benchmarkResult.baseline.sheetsEliminated))
                        .arg(static_cast<qulonglong>(benchmarkResult.baseline.optimizerPasses))
                        .arg(benchmarkResult.optimized.milliseconds, 0, 'f', 1)
                        .arg(static_cast<int>(benchmarkResult.optimized.sheets))
                        .arg(static_cast<qulonglong>(benchmarkResult.optimized.candidateChecks))
                        .arg(static_cast<qulonglong>(benchmarkResult.optimized.nfpChecks))
                        .arg(static_cast<qulonglong>(benchmarkResult.optimized.refillMoves))
                        .arg(static_cast<qulonglong>(benchmarkResult.optimized.exchangeAttempts))
                        .arg(static_cast<qulonglong>(benchmarkResult.optimized.sheetsEliminated))
                        .arg(static_cast<qulonglong>(benchmarkResult.optimized.optimizerPasses))
                );

                const double timeDelta =
                    benchmarkResult.optimized.milliseconds -
                    benchmarkResult.baseline.milliseconds;
                const double timeChangePercent =
                    benchmarkResult.baseline.milliseconds > 1e-9
                        ? (
                            timeDelta /
                            benchmarkResult.baseline.milliseconds
                        ) * 100.0
                        : 0.0;
                const long long sheetDelta =
                    static_cast<long long>(
                        benchmarkResult.optimized.sheets
                    ) -
                    static_cast<long long>(
                        benchmarkResult.baseline.sheets
                    );
                const long long placedDelta =
                    static_cast<long long>(
                        benchmarkResult.optimized.placed
                    ) -
                    static_cast<long long>(
                        benchmarkResult.baseline.placed
                    );
                const double utilizationDelta =
                    (
                        benchmarkResult.optimized.utilization -
                        benchmarkResult.baseline.utilization
                    ) * 100.0;

                appendLog(
                    QString(
                        "Benchmark итог: время %1%2%, листов %3%4, "
                        "размещено %5%6, использование %7%8 п.п."
                    )
                        .arg(
                            timeChangePercent >= 0.0 ? "+" : ""
                        )
                        .arg(
                            timeChangePercent,
                            0,
                            'f',
                            1
                        )
                        .arg(
                            sheetDelta >= 0 ? "+" : ""
                        )
                        .arg(
                            sheetDelta
                        )
                        .arg(
                            placedDelta >= 0 ? "+" : ""
                        )
                        .arg(
                            placedDelta
                        )
                        .arg(
                            utilizationDelta >= 0.0 ? "+" : ""
                        )
                        .arg(
                            utilizationDelta,
                            0,
                            'f',
                            2
                        )
                );
            } catch (const std::exception& error) {
                QMessageBox::critical(
                    this,
                    "Ошибка benchmark",
                    QString::fromUtf8(error.what())
                );
            }
            setBusy(false);
        }
    );
}

void MainWindow::importDxf() {
    const QString fileName = QFileDialog::getOpenFileName(
        this,
        "Открыть DXF",
        QString(),
        "DXF files (*.dxf);;All files (*)"
    );

    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(
            this,
            "Ошибка DXF",
            "Не удалось открыть файл."
        );
        return;
    }

    const QByteArray data = file.readAll();
    const std::string text(data.constData(),
                           static_cast<std::size_t>(data.size()));

    document_ = sheetnest::importDxf(text, 0.25);
    parts_ = partsFromDxf(document_);
    currentFile_ = fileName;
    populatePartTable();

    fileLabel_->setText(
        QString("%1\nКонтуры: %2")
            .arg(fileName)
            .arg(static_cast<int>(document_.contours.size()))
    );

    appendLog(
        QString("DXF: %1, entities=%2, supported=%3, loops=%4.")
            .arg(fileName)
            .arg(static_cast<int>(document_.entitiesRead))
            .arg(static_cast<int>(document_.supportedEntities))
            .arg(static_cast<int>(document_.closedLoopsFound))
    );

    for (const auto& diagnostic : document_.diagnostics) {
        appendLog(
            QString("[%1] %2: %3")
                .arg(severityName(diagnostic.severity))
                .arg(QString::fromStdString(diagnostic.stage))
                .arg(QString::fromStdString(diagnostic.message))
        );
    }

    if (document_.contours.empty()) {
        parts_.clear();
        if (partTable_) partTable_->setRowCount(0);
        instances_.clear();
        partCountLabel_->setText("Деталей: 0");
        calculateButton_->setEnabled(false);
        QMessageBox::warning(
            this,
            "DXF",
            "В файле не найдено ни одного замкнутого контура."
        );
        return;
    }

    calculateButton_->setEnabled(true);
    refreshInstances();
}

void MainWindow::refreshInstances() {
    resetAdaptiveRepairAnimation();
    hasBenchmarkResult_ = false;
    validation_ = {};
    validation_.valid = false;
    if (repairRoundCombo_) {
        QSignalBlocker blocker(repairRoundCombo_);
        repairRoundCombo_->clear();
        repairRoundCombo_->addItem("Итоговая раскладка", -1);
        repairRoundCombo_->setEnabled(false);
    }
    if (repairConflictLayer_) repairConflictLayer_->setEnabled(false);
    if (repairExtractedLayer_) repairExtractedLayer_->setEnabled(false);
    if (repairMovedLayer_) repairMovedLayer_->setEnabled(false);
    if (repairStationaryLayer_) repairStationaryLayer_->setEnabled(false);
    if (repairButton_) {
        repairButton_->setEnabled(false);
    }
    if (exportButton_) {
        exportButton_->setEnabled(false);
    }
    if (benchmarkExportButton_) {
        benchmarkExportButton_->setEnabled(false);
    }

    if (parts_.empty()) {
        instances_.clear();
        partCountLabel_->setText("Деталей: 0");
        return;
    }

    std::vector<std::size_t> quantities(parts_.size(), 1);

    for (std::size_t i = 0; i < parts_.size(); ++i) {
        if (!partTable_ ||
            i >= static_cast<std::size_t>(partTable_->rowCount())) {
            continue;
        }

        const auto* item = partTable_->item(
            static_cast<int>(i),
            3
        );

        bool ok = false;
        const int value = item ? item->text().toInt(&ok) : 1;
        quantities[i] = ok
            ? static_cast<std::size_t>(std::max(0, value))
            : 1;
    }

    instances_ = instancesFromDxf(
        document_,
        quantities
    );

    partCountLabel_->setText(
        QString("Деталей: %1")
            .arg(static_cast<int>(instances_.size()))
    );
}

void MainWindow::populatePartTable() {
    if (!partTable_) return;

    QSignalBlocker blocker(partTable_);
    partTable_->setRowCount(static_cast<int>(parts_.size()));

    for (std::size_t i = 0; i < parts_.size(); ++i) {
        const auto& part = parts_[i];

        auto* idItem = new QTableWidgetItem(
            QString::fromStdString(part.id)
        );
        auto* layerItem = new QTableWidgetItem(
            QString::fromStdString(part.layer)
        );
        auto* sourceItem = new QTableWidgetItem(
            QString::fromStdString(part.sourceId)
        );

        idItem->setFlags(
            idItem->flags() & ~Qt::ItemIsEditable
        );
        layerItem->setFlags(
            layerItem->flags() & ~Qt::ItemIsEditable
        );
        sourceItem->setFlags(
            sourceItem->flags() & ~Qt::ItemIsEditable
        );

        partTable_->setItem(
            static_cast<int>(i), 0, idItem
        );
        partTable_->setItem(
            static_cast<int>(i), 1, layerItem
        );
        partTable_->setItem(
            static_cast<int>(i), 2, sourceItem
        );

        auto* quantity = new QTableWidgetItem("1");
        quantity->setTextAlignment(Qt::AlignCenter);
        partTable_->setItem(
            static_cast<int>(i), 3,
            quantity
        );
    }

    refreshInstances();
}

void MainWindow::populateDiagnostics() {
    if (!diagnosticsTable_) return;

    const auto diagnostics = diagnoseNest(
        instances_,
        result_
    );

    diagnosticsTable_->setRowCount(
        static_cast<int>(diagnostics.size())
    );

    auto statusText = [](InstanceDiagnosticStatus status) {
        switch (status) {
        case InstanceDiagnosticStatus::Placed:
            return QString("РАЗМЕЩЕНО");
        case InstanceDiagnosticStatus::Unplaced:
            return QString("НЕ РАЗМЕЩЕНО");
        case InstanceDiagnosticStatus::Unknown:
            return QString("UNKNOWN");
        }
        return QString("UNKNOWN");
    };

    for (std::size_t i = 0; i < diagnostics.size(); ++i) {
        const auto& d = diagnostics[i];

        const QString values[] = {
            QString::fromStdString(d.instanceId),
            QString::fromStdString(d.unitId),
            QString::fromStdString(d.sourceId),
            QString::fromStdString(d.layer),
            statusText(d.status) + " / " +
                QString::fromStdString(d.stage),
            QString::fromStdString(d.message)
        };

        for (int column = 0; column < 6; ++column) {
            diagnosticsTable_->setItem(
                static_cast<int>(i),
                column,
                new QTableWidgetItem(values[column])
            );
        }
    }

    diagnosticsTable_->resizeColumnsToContents();
}

void MainWindow::populateProductionValidation() {
    if (!validatorTable_) return;

    validatorTable_->setRowCount(
        static_cast<int>(validation_.issues.size())
    );

    for (std::size_t i = 0; i < validation_.issues.size(); ++i) {
        const auto& issue = validation_.issues[i];

        const QString values[] = {
            QString::fromUtf8(
                productionValidationIssueTypeName(issue.type)
            ),
            QString::number(
                static_cast<qulonglong>(issue.sheetIndex + 1)
            ),
            QString::fromStdString(issue.instanceId),
            QString::fromStdString(issue.relatedInstanceId),
            QString::number(issue.measuredMm, 'f', 3),
            QString::number(issue.requiredMm, 'f', 3),
            QString::fromStdString(issue.message)
        };

        for (int column = 0; column < 7; ++column) {
            validatorTable_->setItem(
                static_cast<int>(i),
                column,
                new QTableWidgetItem(values[column])
            );
        }
    }

    if (validation_.valid && validatorTable_->rowCount() == 0) {
        validatorTable_->setRowCount(1);

        const QString values[] = {
            "OK", "-", "-", "-", "-", "-",
            "Production Validator: ошибок не обнаружено."
        };

        for (int column = 0; column < 7; ++column) {
            validatorTable_->setItem(
                0,
                column,
                new QTableWidgetItem(values[column])
            );
        }
    }

    validatorTable_->resizeColumnsToContents();
}

void MainWindow::populateBenchmark(
    const BenchmarkResult& benchmarkResult
) {
    if (!benchmarkTable_) return;

    benchmarkTable_->setRowCount(2);

    const BenchmarkCase rows[] = {
        benchmarkResult.baseline,
        benchmarkResult.optimized
    };

    for (int row = 0; row < 2; ++row) {
        const auto& b = rows[row];

        const QString values[] = {
            QString::fromStdString(b.name),
            QString::number(b.milliseconds, 'f', 1),
            QString::number(static_cast<qulonglong>(b.sheets)),
            QString::number(static_cast<qulonglong>(b.placed)),
            QString::number(static_cast<qulonglong>(b.skipped)),
            QString("%1%").arg(b.utilization * 100.0, 0, 'f', 2),
            QString::number(static_cast<qulonglong>(b.candidateChecks)),
            QString::number(static_cast<qulonglong>(b.collisionChecks)),
            QString::number(static_cast<qulonglong>(b.nfpChecks)),
            QString::number(static_cast<qulonglong>(b.refillMoves)),
            QString::number(static_cast<qulonglong>(b.exchangeAttempts)),
            QString::number(static_cast<qulonglong>(b.sheetsEliminated)),
            QString::number(static_cast<qulonglong>(b.optimizerPasses)),
            QString::number(static_cast<qulonglong>(b.nfpTimeouts)),
            QString::number(static_cast<qulonglong>(b.nfpComplexityFallbacks))
        };

        for (int column = 0; column < 15; ++column) {
            benchmarkTable_->setItem(
                row,
                column,
                new QTableWidgetItem(values[column])
            );
        }
    }

    benchmarkTable_->resizeColumnsToContents();
}

void MainWindow::calculate() {
    if (instances_.empty()) {
        QMessageBox::information(
            this,
            "Расчёт",
            "Сначала загрузите DXF с деталями."
        );
        return;
    }

    std::vector<int> rotations;
    if (rotation0_->isChecked()) rotations.push_back(0);
    if (rotation90_->isChecked()) rotations.push_back(90);
    if (rotation180_->isChecked()) rotations.push_back(180);
    if (rotation270_->isChecked()) rotations.push_back(270);

    if (rotations.empty()) {
        QMessageBox::warning(
            this,
            "Повороты",
            "Выберите хотя бы один угол."
        );
        return;
    }

    sheet_ = {
        sheetWidthSpin_->value(),
        sheetHeightSpin_->value(),
        marginSpin_->value()
    };

    options_.rotations = rotations;
    options_.iterations =
        static_cast<std::size_t>(iterationsSpin_->value());
    options_.gapMm = gapSpin_->value();
    options_.enableAutoRepair = true;

    if (repairRequested_) {
        options_.iterations =
            std::clamp<std::size_t>(
                std::max<std::size_t>(
                    64,
                    options_.iterations * 2
                ),
                64,
                128
            );
        options_.seed += 0xA5A5F00Du;

        appendLog(
            QString("Запущено усиленное исправление: %1 итераций + Auto Repair.")
                .arg(static_cast<int>(options_.iterations))
        );
    }

    Material material = Material::CarbonSteel;
    switch (materialCombo_->currentIndex()) {
    case 1: material = Material::StainlessSteel; break;
    case 2: material = Material::Aluminum; break;
    case 3: material = Material::Brass; break;
    default: break;
    }

    technology_ = bodor3kWParameters(
        material,
        thicknessSpin_->value()
    );

    validation_ = {};
    validation_.valid = false;
    if (repairRoundCombo_) {
        QSignalBlocker blocker(repairRoundCombo_);
        repairRoundCombo_->clear();
        repairRoundCombo_->addItem("Итоговая раскладка", -1);
        repairRoundCombo_->setEnabled(false);
    }
    if (repairConflictLayer_) repairConflictLayer_->setEnabled(false);
    if (repairExtractedLayer_) repairExtractedLayer_->setEnabled(false);
    if (repairMovedLayer_) repairMovedLayer_->setEnabled(false);
    if (repairStationaryLayer_) repairStationaryLayer_->setEnabled(false);
    exportButton_->setEnabled(false);

    const auto instancesCopy = instances_;
    const auto sheetCopy = sheet_;
    const auto optionsCopy = options_;
    const auto technologyCopy = technology_;

    auto controller =
        std::make_shared<sheetnest::ParallelNestingController>();
    nestingController_ = controller;

    sheetnest::ParallelNestingOptions parallelOptions;
    parallelOptions.workers =
        static_cast<std::size_t>(workersSpin_->value());
    parallelOptions.iterations = options_.iterations;
    parallelOptions.timeBudgetMs =
        static_cast<std::uint64_t>(timeBudgetSpin_->value()) * 1000u;

    QPointer<MainWindow> safeThis(this);
    parallelOptions.onProgress =
        [safeThis](const sheetnest::NestingProgress& event) {
            if (!safeThis) return;

            QMetaObject::invokeMethod(
                safeThis,
                [safeThis, event]() {
                    if (!safeThis) return;
                    safeThis->updateProgress(event);
                },
                Qt::QueuedConnection
            );
        };

    pauseLaserAnimation();
    resetLaserAnimation();
    if (laserOperationCombo_) {
        laserOperationCombo_->clear();
    }

    setBusy(true);
    calculationStartedMs_ = QDateTime::currentMSecsSinceEpoch();
    lastProgressMs_ = calculationStartedMs_;
    watchdogTriggered_ = false;
    userCancelRequested_ = false;
    if (calculationWatchdog_) {
        calculationWatchdog_->start();
    }
    progress_->setRange(0, 100);
    progress_->setValue(0);
    progressDetails_->setText(
        QString("Контроллер: %1 workers, лимит %2 с")
            .arg(workersSpin_->value() == 0
                ? QString("авто")
                : QString::number(workersSpin_->value()))
            .arg(timeBudgetSpin_->value())
    );
    appendLog(
        QString("Запущен Parallel Nesting Engine: workers=%1, итераций=%2, лимит=%3 с.")
            .arg(workersSpin_->value() == 0
                ? QString("auto")
                : QString::number(workersSpin_->value()))
            .arg(static_cast<int>(options_.iterations))
            .arg(timeBudgetSpin_->value())
    );

    watcher_->setFuture(
        QtConcurrent::run(
            [this,
             instancesCopy,
             sheetCopy,
             optionsCopy,
             technologyCopy,
             parallelOptions,
             controller]() {
                return performCalculation(
                    instancesCopy,
                    sheetCopy,
                    optionsCopy,
                    technologyCopy,
                    parallelOptions,
                    controller
                );
            }
        )
    );
}

void MainWindow::repairErrors() {
    if (instances_.empty() || result_.sheets.empty()) {
        return;
    }

    if (validation_.valid) {
        QMessageBox::information(
            this,
            "Production Validator",
            "Ошибок для исправления не обнаружено."
        );
        return;
    }

    repairRequested_ = true;
    appendLog(
        "Запущено автоматическое исправление: усиленный пересчёт → "
        "Global Optimizer → Production Validator."
    );
    calculate();
}

void MainWindow::benchmark() {
    if (instances_.empty()) {
        QMessageBox::information(
            this,
            "Benchmark",
            "Сначала загрузите DXF и задайте количество деталей."
        );
        return;
    }

    std::vector<int> rotations;
    if (rotation0_->isChecked()) rotations.push_back(0);
    if (rotation90_->isChecked()) rotations.push_back(90);
    if (rotation180_->isChecked()) rotations.push_back(180);
    if (rotation270_->isChecked()) rotations.push_back(270);

    if (rotations.empty()) {
        QMessageBox::warning(
            this,
            "Benchmark",
            "Выберите хотя бы один угол."
        );
        return;
    }

    sheet_ = {
        sheetWidthSpin_->value(),
        sheetHeightSpin_->value(),
        marginSpin_->value()
    };

    options_.rotations = rotations;
    options_.iterations =
        static_cast<std::size_t>(iterationsSpin_->value());
    options_.gapMm = gapSpin_->value();

    const auto instancesCopy = instances_;
    const auto sheetCopy = sheet_;
    const auto optionsCopy = options_;

    hasBenchmarkResult_ = false;
    validation_ = {};
    if (benchmarkExportButton_) {
        benchmarkExportButton_->setEnabled(false);
    }
    nestingController_.reset();
    setBusy(true);
    appendLog("Запущен benchmark: базовый поиск vs оптимизированный...");

    benchmarkWatcher_->setFuture(
        QtConcurrent::run(
            [this, instancesCopy, sheetCopy, optionsCopy]() {
                return performBenchmark(
                    instancesCopy,
                    sheetCopy,
                    optionsCopy
                );
            }
        )
    );
}

CalculationOutput MainWindow::performCalculation(
    std::vector<Instance> instances,
    Sheet sheet,
    Options options,
    CuttingParameters technology,
    ParallelNestingOptions parallelOptions,
    std::shared_ptr<ParallelNestingController> controller
) const {
    CalculationOutput output;
    output.technology = technology;
    bool controllerProvidedValidation = false;

    parallelOptions.onValidation =
        [&output, &controllerProvidedValidation](
            const sheetnest::ProductionValidationReport& report
        ) {
            output.validation = report;
            controllerProvidedValidation = true;
        };

    output.result = controller
        ? controller->run(
            instances,
            sheet,
            options,
            parallelOptions
        )
        : nest(
            instances,
            sheet,
            options
        );

    if (!controllerProvidedValidation) {
        output.validation = validateProductionResult(
            instances,
            sheet,
            options,
            output.result
        );
    }

    output.cuttingRoute = planWholeResultRoute(output.result, instances, technology);
    PathOptions routeOptions;
    routeOptions.rapidSpeedMMin = 120.0;
    routeOptions.pierceSeconds = 0.25;
    output.cutting = estimateCuttingPath(output.cuttingRoute, technology, routeOptions);
    output.diagnostics = diagnoseNest(
        instances,
        output.result
    );
    return output;
}

sheetnest::BenchmarkResult MainWindow::performBenchmark(
    std::vector<Instance> instances,
    Sheet sheet,
    Options options
) const {
    return benchmarkNest(
        instances,
        sheet,
        options
    );
}

void MainWindow::exportBenchmarkResults() {
    if (!hasBenchmarkResult_) {
        return;
    }

    const QString fileName = QFileDialog::getSaveFileName(
        this,
        "Сохранить результаты Benchmark",
        currentFile_.isEmpty()
            ? "sheetnest-benchmark.csv"
            : QFileInfo(currentFile_).completeBaseName() + "_benchmark.csv",
        "CSV files (*.csv);;JSON files (*.json)"
    );

    if (fileName.isEmpty()) return;

    auto appendCsvField = [](QString& row, const QString& value) {
        QString escaped = value;
        escaped.replace('"', "\"\"" );
        row += '"';
        row += escaped;
        row += '"';
    };

    auto benchmarkObject = [](const BenchmarkCase& b) {
        QJsonObject object;
        object["name"] = QString::fromStdString(b.name);
        object["milliseconds"] = b.milliseconds;
        object["sheets"] = static_cast<qint64>(b.sheets);
        object["placed"] = static_cast<qint64>(b.placed);
        object["skipped"] = static_cast<qint64>(b.skipped);
        object["utilization"] = b.utilization;
        object["candidateChecks"] = static_cast<qint64>(b.candidateChecks);
        object["collisionChecks"] = static_cast<qint64>(b.collisionChecks);
        object["nfpChecks"] = static_cast<qint64>(b.nfpChecks);
        object["refillMoves"] = static_cast<qint64>(b.refillMoves);
        object["exchangeAttempts"] = static_cast<qint64>(b.exchangeAttempts);
        object["sheetsEliminated"] = static_cast<qint64>(b.sheetsEliminated);
        object["optimizerPasses"] = static_cast<qint64>(b.optimizerPasses);
        return object;
    };

    const QString suffix = QFileInfo(fileName).suffix().toLower();
    QSaveFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(
            this,
            "Экспорт Benchmark",
            "Не удалось открыть файл для записи."
        );
        return;
    }

    if (suffix == "json") {
        QJsonObject root;
        root["exportedAt"] =
            QDateTime::currentDateTime().toString(Qt::ISODate);
        root["baseline"] = benchmarkObject(lastBenchmarkResult_.baseline);
        root["optimized"] = benchmarkObject(lastBenchmarkResult_.optimized);

        const QByteArray data =
            QJsonDocument(root).toJson(QJsonDocument::Indented);
        file.write(data);
    } else {
        QString csv;
        csv += "mode,time_ms,sheets,placed,skipped,utilization_percent,candidateChecks,collisionChecks,nfpChecks,refillMoves,exchangeAttempts,sheetsEliminated,optimizerPasses\n";

        const BenchmarkCase rows[] = {
            lastBenchmarkResult_.baseline,
            lastBenchmarkResult_.optimized
        };

        for (const auto& b : rows) {
            QString row;
            const QString values[] = {
                QString::fromStdString(b.name),
                QString::number(b.milliseconds, 'f', 3),
                QString::number(static_cast<qulonglong>(b.sheets)),
                QString::number(static_cast<qulonglong>(b.placed)),
                QString::number(static_cast<qulonglong>(b.skipped)),
                QString::number(b.utilization * 100.0, 'f', 4),
                QString::number(static_cast<qulonglong>(b.candidateChecks)),
                QString::number(static_cast<qulonglong>(b.collisionChecks)),
                QString::number(static_cast<qulonglong>(b.nfpChecks)),
                QString::number(static_cast<qulonglong>(b.refillMoves)),
                QString::number(static_cast<qulonglong>(b.exchangeAttempts)),
                QString::number(static_cast<qulonglong>(b.sheetsEliminated)),
                QString::number(static_cast<qulonglong>(b.optimizerPasses))
            };

            for (int i = 0; i < 13; ++i) {
                if (i > 0) row += ',';
                appendCsvField(row, values[i]);
            }
            row += '\n';
            csv += row;
        }

        file.write(csv.toUtf8());
    }

    if (!file.commit()) {
        QMessageBox::critical(
            this,
            "Экспорт Benchmark",
            "Не удалось завершить запись файла."
        );
        return;
    }

    appendLog(QString("Результаты Benchmark сохранены: %1").arg(fileName));
    statusBar()->showMessage("Benchmark экспортирован", 5000);
}

// Production Validator: final export gate
void MainWindow::exportDxf() {
    if (result_.sheets.empty()) {
        return;
    }

    if (!validation_.valid) {
        QMessageBox::warning(
            this,
            "Production Validator",
            "Раскладка не прошла Production Validator. "
            "Экспорт DXF заблокирован до устранения ошибок."
        );
        return;
    }

    const QString fileName = QFileDialog::getSaveFileName(
        this,
        "Сохранить раскладку DXF",
        currentFile_.isEmpty()
            ? "sheetnest-layout.dxf"
            : QFileInfo(currentFile_).completeBaseName() + "_layout.dxf",
        "DXF files (*.dxf)"
    );

    if (fileName.isEmpty()) return;

    const std::string text = exportNestDxf(
        result_,
        instances_,
        sheet_
    );

    const QByteArray payload(
        text.data(),
        static_cast<int>(text.size())
    );

    QSaveFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(
            this,
            "Экспорт DXF",
            "Не удалось открыть файл для безопасной записи."
        );
        return;
    }

    const qint64 written = file.write(payload);
    if (written != payload.size()) {
        file.cancelWriting();
        QMessageBox::critical(
            this,
            "Экспорт DXF",
            "Не удалось полностью записать DXF-файл."
        );
        return;
    }

    if (!file.commit()) {
        QMessageBox::critical(
            this,
            "Экспорт DXF",
            "Не удалось завершить безопасную запись DXF-файла."
        );
        return;
    }

    appendLog(
        QString("Раскладка сохранена: %1 • %2 байт")
            .arg(fileName)
            .arg(payload.size())
    );
    statusBar()->showMessage(
        "DXF экспортирован",
        5000
    );
}

void MainWindow::updateTechnologyPreview() {
    Material material = Material::CarbonSteel;
    switch (materialCombo_->currentIndex()) {
    case 1: material = Material::StainlessSteel; break;
    case 2: material = Material::Aluminum; break;
    case 3: material = Material::Brass; break;
    default: break;
    }

    technology_ = bodor3kWParameters(
        material,
        thicknessSpin_->value()
    );

    if (technology_.speedMMin <= 0.0) {
        techLabel_->setText(
            "Для выбранного материала/толщины скорость в технологической "
            "таблице не найдена."
        );
        return;
    }

    techLabel_->setText(
        QString("%1 м/мин • %2")
            .arg(technology_.speedMMin, 0, 'f', 2)
            .arg(QString::fromStdString(technology_.assistGas))
    );
}

QString MainWindow::materialName(Material material) const {
    switch (material) {
    case Material::CarbonSteel: return "Конструкционная сталь";
    case Material::StainlessSteel: return "Нержавеющая сталь";
    case Material::Aluminum: return "Алюминий";
    case Material::Brass: return "Латунь";
    case Material::MildSteel: return "Мягкая сталь";
    }
    return "Материал";
}

void MainWindow::appendLog(const QString& text) {
    log_->appendPlainText(text);
}

void MainWindow::updateProgress(
    const sheetnest::NestingProgress& progress
) {
    lastProgressMs_ = QDateTime::currentMSecsSinceEpoch();
    if (progress.totalIterations > 0) {
        const auto completed =
            std::min(
                progress.completedIterations,
                progress.totalIterations
            );
        const int percent =
            static_cast<int>(
                (completed * 100u) /
                progress.totalIterations
            );
        progress_->setRange(0, 100);
        progress_->setValue(percent);
    }

    const int remainingSeconds =
        static_cast<int>(progress.remainingMs / 1000u);

    progressDetails_->setText(
        QString("%1\nWorkers: %2\nИтерации: %3 / %4\n"
                "Листов: %5 • размещено: %6 • пропущено: %7\n"
                "Прошло: %8 с • осталось: %9 с")
            .arg(QString::fromStdString(progress.message))
            .arg(static_cast<int>(progress.workerCount))
            .arg(static_cast<int>(progress.completedIterations))
            .arg(static_cast<int>(progress.totalIterations))
            .arg(static_cast<int>(progress.sheets))
            .arg(static_cast<int>(progress.placed))
            .arg(static_cast<int>(progress.skipped))
            .arg(static_cast<int>(progress.elapsedMs / 1000u))
            .arg(remainingSeconds)
    );

    statusBar()->showMessage(
        QString("%1 | %2/%3 итераций | %4 листов")
            .arg(QString::fromStdString(progress.message))
            .arg(static_cast<int>(progress.completedIterations))
            .arg(static_cast<int>(progress.totalIterations))
            .arg(static_cast<int>(progress.sheets))
    );

    switch (progress.phase) {
    case sheetnest::NestingProgressPhase::Starting:
        break;

    case sheetnest::NestingProgressPhase::WorkerStarted:
        break;

    case sheetnest::NestingProgressPhase::IterationFinished:
        break;

    case sheetnest::NestingProgressPhase::CandidatesCollected:
        appendLog(
            QString("Candidate Collector: %1")
                .arg(QString::fromStdString(progress.message))
        );
        break;

    case sheetnest::NestingProgressPhase::GlobalOptimization:
        appendLog(
            QString("Global Optimizer: %1")
                .arg(QString::fromStdString(progress.message))
        );
        break;

    case sheetnest::NestingProgressPhase::ProductionValidation:
        appendLog(
            QString("Production Validator: %1")
                .arg(QString::fromStdString(progress.message))
        );
        break;

    case sheetnest::NestingProgressPhase::Completed:
        progress_->setValue(100);
        appendLog(
            QString("Parallel Nesting: завершено %1/%2 итераций.")
                .arg(static_cast<int>(progress.completedIterations))
                .arg(static_cast<int>(progress.totalIterations))
        );
        break;

    case sheetnest::NestingProgressPhase::Cancelled:
        appendLog(
            QString("Parallel Nesting остановлен пользователем после %1/%2 итераций.")
                .arg(static_cast<int>(progress.completedIterations))
                .arg(static_cast<int>(progress.totalIterations))
        );
        break;

    case sheetnest::NestingProgressPhase::TimedOut:
        appendLog(
            QString("Parallel Nesting остановлен по лимиту времени после %1/%2 итераций.")
                .arg(static_cast<int>(progress.completedIterations))
                .arg(static_cast<int>(progress.totalIterations))
        );
        break;
    }
}

void MainWindow::resetAdaptiveRepairAnimation() {
    if (repairAnimationTimer_) {
        repairAnimationTimer_->stop();
    }

    repairAnimationPlaying_ = false;
    repairAnimationSession_ = false;
    repairAnimationFrame_ = -1;
    updateAdaptiveRepairAnimationUi();
}

void MainWindow::toggleAdaptiveRepairAnimation() {
    if (validation_.adaptiveHistory.empty()) {
        return;
    }

    const int totalFrames =
        static_cast<int>(
            validation_.adaptiveHistory.size() * 4
        );

    if (!repairAnimationSession_) {
        int selectedRound =
            repairRoundCombo_
                ? repairRoundCombo_->currentData().toInt()
                : -1;

        if (selectedRound > 0) {
            repairAnimationFrame_ =
                std::clamp(
                    (selectedRound - 1) * 4,
                    0,
                    totalFrames - 1
                );
        } else {
            repairAnimationFrame_ = 0;
        }

        repairAnimationSession_ = true;
    }

    if (repairAnimationFrame_ >= totalFrames - 1) {
        repairAnimationFrame_ = 0;
    }

    repairAnimationPlaying_ = true;
    updateAdaptiveRepairAnimationUi();

    if (repairAnimationTimer_) {
        repairAnimationTimer_->start();
    }

    refreshAdaptiveRepairView();
}

void MainWindow::pauseAdaptiveRepairAnimation() {
    repairAnimationPlaying_ = false;

    if (repairAnimationTimer_) {
        repairAnimationTimer_->stop();
    }

    updateAdaptiveRepairAnimationUi();
}

void MainWindow::stepAdaptiveRepairAnimation(int direction) {
    if (validation_.adaptiveHistory.empty() ||
        direction == 0) {
        return;
    }

    const int totalFrames =
        static_cast<int>(
            validation_.adaptiveHistory.size() * 4
        );

    if (!repairAnimationSession_) {
        int selectedRound =
            repairRoundCombo_
                ? repairRoundCombo_->currentData().toInt()
                : -1;

        repairAnimationFrame_ =
            selectedRound > 0
                ? std::clamp(
                    (selectedRound - 1) * 4,
                    0,
                    totalFrames - 1
                )
                : 0;
        repairAnimationSession_ = true;
    }

    repairAnimationPlaying_ = false;

    if (repairAnimationTimer_) {
        repairAnimationTimer_->stop();
    }

    repairAnimationFrame_ =
        std::clamp(
            repairAnimationFrame_ + direction,
            0,
            totalFrames - 1
        );

    updateAdaptiveRepairAnimationUi();
    refreshAdaptiveRepairView();
}

void MainWindow::advanceAdaptiveRepairAnimation() {
    if (!repairAnimationPlaying_ ||
        validation_.adaptiveHistory.empty()) {
        return;
    }

    const int totalFrames =
        static_cast<int>(
            validation_.adaptiveHistory.size() * 4
        );

    ++repairAnimationFrame_;

    if (repairAnimationFrame_ >= totalFrames) {
        repairAnimationFrame_ = totalFrames - 1;
        repairAnimationPlaying_ = false;

        if (repairAnimationTimer_) {
            repairAnimationTimer_->stop();
        }
    }

    updateAdaptiveRepairAnimationUi();
    refreshAdaptiveRepairView();
}

void MainWindow::updateAdaptiveRepairAnimationUi() {
    const bool hasHistory =
        !validation_.adaptiveHistory.empty();

    if (!hasHistory) {
        if (repairPlayButton_) repairPlayButton_->setEnabled(false);
        if (repairPauseButton_) repairPauseButton_->setEnabled(false);
        if (repairPrevButton_) repairPrevButton_->setEnabled(false);
        if (repairNextButton_) repairNextButton_->setEnabled(false);
        if (repairSpeedCombo_) repairSpeedCombo_->setEnabled(false);
        if (repairStageLabel_) {
            repairStageLabel_->setText("Анимация: нет истории");
        }
        return;
    }

    if (repairPlayButton_) {
        repairPlayButton_->setEnabled(!repairAnimationPlaying_);
    }
    if (repairPauseButton_) {
        repairPauseButton_->setEnabled(repairAnimationPlaying_);
    }
    if (repairPrevButton_) {
        repairPrevButton_->setEnabled(true);
    }
    if (repairNextButton_) {
        repairNextButton_->setEnabled(true);
    }
    if (repairSpeedCombo_) {
        repairSpeedCombo_->setEnabled(true);
    }

    if (repairSpeedCombo_ &&
        repairAnimationTimer_) {
        const double speed =
            repairSpeedCombo_->currentData().toDouble();
        const int interval =
            std::max(
                150,
                static_cast<int>(
                    900.0 /
                    std::max(0.25, speed)
                )
            );
        repairAnimationTimer_->setInterval(interval);
    }

    if (!repairAnimationSession_ ||
        repairAnimationFrame_ < 0) {
        if (repairStageLabel_) {
            repairStageLabel_->setText(
                QString("Анимация: готово • %1 раундов")
                    .arg(
                        static_cast<qulonglong>(
                            validation_.adaptiveHistory.size()
                        )
                    )
            );
        }
        return;
    }

    const int roundIndex =
        repairAnimationFrame_ / 4 + 1;
    const int stage =
        repairAnimationFrame_ % 4;

    const QString stages[] = {
        "Конфликт",
        "Извлечение деталей",
        "Локальная перепаковка",
        "Production Validator"
    };

    if (repairRoundCombo_) {
        const QSignalBlocker blocker(repairRoundCombo_);
        const int index =
            repairRoundCombo_->findData(roundIndex);
        if (index >= 0) {
            repairRoundCombo_->setCurrentIndex(index);
        }
    }

    if (repairStageLabel_) {
        repairStageLabel_->setText(
            QString("Раунд %1 • этап %2/4: %3%4")
                .arg(roundIndex)
                .arg(stage + 1)
                .arg(stages[stage])
                .arg(
                    repairAnimationPlaying_
                        ? " • воспроизведение"
                        : ""
                )
        );
    }
}

void MainWindow::refreshAdaptiveRepairView() {
    if (!view_) return;

    int selectedRound = -1;
    if (repairRoundCombo_) {
        selectedRound =
            repairRoundCombo_->currentData().toInt();
    }

    int animationRound = selectedRound;
    int animationStage = -1;

    if (repairAnimationSession_ &&
        repairAnimationFrame_ >= 0) {
        animationRound =
            repairAnimationFrame_ / 4 + 1;
        animationStage =
            repairAnimationFrame_ % 4;
    }

    if (laserAnimationOperation_ >= 0) {
        view_->setCuttingAnimationOperationProgress(
            laserAnimationOperation_,
            laserContourProgress_
        );
    } else {
        view_->setCuttingAnimationProgress(
            laserAnimationProgress_
        );
    }

    view_->setCuttingRoute(cuttingRoute_, sheet_.height);
    view_->showResult(
        result_,
        instances_,
        sheet_,
        &validation_,
        animationRound,
        repairConflictLayer_
            ? repairConflictLayer_->isChecked()
            : true,
        repairExtractedLayer_
            ? repairExtractedLayer_->isChecked()
            : true,
        repairMovedLayer_
            ? repairMovedLayer_->isChecked()
            : true,
        repairStationaryLayer_
            ? repairStationaryLayer_->isChecked()
            : true,
        animationStage
    );
}

void MainWindow::populateLaserOperationSelector() {
    if (!laserOperationCombo_ || !view_) return;

    const QSignalBlocker blocker(laserOperationCombo_);
    laserOperationCombo_->clear();

    const auto& operations = view_->cuttingRouteOperations();
    for (const auto& op : operations) {
        const QString type =
            op.inner ? "внутренний контур" : "внешний контур";
        const QString detail =
            QString::fromStdString(op.instanceId);

        const QString label =
            op.inner
                ? QString(
                    "Операция %1 • лист %2 • %3 • отверстие %4 • %5 • %6 мм"
                )
                    .arg(static_cast<qulonglong>(op.operation + 1))
                    .arg(static_cast<qulonglong>(op.sheetIndex + 1))
                    .arg(detail)
                    .arg(static_cast<qulonglong>(op.contourIndex + 1))
                    .arg(type)
                    .arg(op.cutLengthMm, 0, 'f', 1)
                : QString(
                    "Операция %1 • лист %2 • %3 • внешний контур • %4 мм"
                )
                    .arg(static_cast<qulonglong>(op.operation + 1))
                    .arg(static_cast<qulonglong>(op.sheetIndex + 1))
                    .arg(detail)
                    .arg(op.cutLengthMm, 0, 'f', 1);

        laserOperationCombo_->addItem(
            label,
            static_cast<int>(op.operation)
        );
    }

    laserAnimationOperation_ = -1;
    laserContourProgress_ = 0.0;
    if (!operations.empty()) {
        laserOperationCombo_->setCurrentIndex(0);
    }
    if (laserContourProgressSlider_) {
        QSignalBlocker sliderBlocker(laserContourProgressSlider_);
        laserContourProgressSlider_->setValue(0);
    }
    updateLaserAnimationUi();
}

void MainWindow::laserSelectOperation(int index) {
    if (!view_) return;

    const auto& operations = view_->cuttingRouteOperations();
    if (index < 0 ||
        static_cast<std::size_t>(index) >= operations.size()) {
        return;
    }

    pauseLaserAnimation();

    laserAnimationOperation_ = index;
    laserAnimationProgress_ = 0.0;
    laserContourProgress_ = 0.0;
    view_->setCuttingAnimationProgress(0.0);
    view_->setCuttingAnimationOperationProgress(index, 0.0);
    if (laserContourProgressSlider_) {
        QSignalBlocker blocker(laserContourProgressSlider_);
        laserContourProgressSlider_->setValue(0);
    }

    updateLaserAnimationUi();
    refreshAdaptiveRepairView();
}

void MainWindow::laserSetContourProgress(double progress) {
    if (!view_ || laserAnimationPlaying_) return;
    if (laserAnimationOperation_ < 0 ||
        static_cast<std::size_t>(laserAnimationOperation_) >=
            view_->cuttingRouteOperations().size()) {
        return;
    }

    laserContourProgress_ = std::clamp(progress, 0.0, 1.0);
    if (laserContourProgressSlider_) {
        QSignalBlocker blocker(laserContourProgressSlider_);
        laserContourProgressSlider_->setValue(
            static_cast<int>(std::lround(laserContourProgress_ * 100.0))
        );
    }

    view_->setCuttingAnimationOperationProgress(
        laserAnimationOperation_,
        laserContourProgress_
    );
    updateLaserAnimationUi();
    refreshAdaptiveRepairView();
}

void MainWindow::laserContourStart() {
    laserSetContourProgress(0.0);
}

void MainWindow::laserContourMiddle() {
    laserSetContourProgress(0.5);
}

void MainWindow::laserContourEnd() {
    laserSetContourProgress(1.0);
}

void MainWindow::laserContourStep(int direction) {
    constexpr double step = 0.05;
    laserSetContourProgress(
        laserContourProgress_ + direction * step
    );
}

void MainWindow::laserPreviousOperation() {
    const auto& operations = view_->cuttingRouteOperations();
    if (operations.empty()) return;

    const int current =
        laserAnimationOperation_ >= 0
            ? laserAnimationOperation_
            : 0;
    const int target = std::max(0, current - 1);

    laserSelectOperation(target);
    if (laserOperationCombo_) {
        QSignalBlocker blocker(laserOperationCombo_);
        laserOperationCombo_->setCurrentIndex(target);
    }
}

void MainWindow::laserNextOperation() {
    const auto& operations = view_->cuttingRouteOperations();
    if (operations.empty()) return;

    const int current =
        laserAnimationOperation_ >= 0
            ? laserAnimationOperation_
            : -1;
    const int target = std::min(
        static_cast<int>(operations.size()) - 1,
        current + 1
    );

    laserSelectOperation(target);
    if (laserOperationCombo_) {
        QSignalBlocker blocker(laserOperationCombo_);
        laserOperationCombo_->setCurrentIndex(target);
    }
}

void MainWindow::toggleLaserAnimation() {
    if (!cuttingRouteCheck_ ||
        !cuttingRouteCheck_->isChecked() ||
        result_.sheets.empty()) {
        return;
    }

    if (laserAnimationProgress_ >= 1.0 - 1e-9) {
        laserAnimationProgress_ = 0.0;
        laserAnimationOperation_ = -1;
        view_->setCuttingAnimationOperation(-1);
    }

    laserAnimationPlaying_ = true;
    if (laserAnimationTimer_) {
        laserAnimationTimer_->start();
    }

    updateLaserAnimationUi();
    refreshAdaptiveRepairView();
}

void MainWindow::pauseLaserAnimation() {
    laserAnimationPlaying_ = false;
    if (laserAnimationTimer_) {
        laserAnimationTimer_->stop();
    }
    updateLaserAnimationUi();
}

void MainWindow::resetLaserAnimation() {
    laserAnimationPlaying_ = false;
    laserAnimationProgress_ = 1.0;
    laserAnimationOperation_ = -1;
    laserContourProgress_ = 0.0;

    if (view_) {
        view_->setCuttingAnimationOperation(-1);
    }

    if (laserOperationCombo_) {
        QSignalBlocker blocker(laserOperationCombo_);
        laserOperationCombo_->setCurrentIndex(-1);
    }

    if (laserAnimationTimer_) {
        laserAnimationTimer_->stop();
    }

    if (view_) {
        view_->setCuttingAnimationProgress(
            laserAnimationProgress_
        );
    }

    updateLaserAnimationUi();
}

void MainWindow::advanceLaserAnimation() {
    if (!laserAnimationPlaying_ ||
        result_.sheets.empty() ||
        !cuttingRouteCheck_ ||
        !cuttingRouteCheck_->isChecked()) {
        pauseLaserAnimation();
        return;
    }

    const double speed =
        laserSpeedCombo_
            ? std::max(
                0.25,
                laserSpeedCombo_->currentData().toDouble()
            )
            : 1.0;

    // About 20 seconds for a complete route at 1×. The animation is a
    // visual replay; the real CAM cutting time is shown separately in the
    // technology/result panel.
    laserAnimationProgress_ +=
        0.005 * speed;

    if (laserAnimationProgress_ >= 1.0) {
        laserAnimationProgress_ = 1.0;
        laserAnimationOperation_ = -1;
        view_->setCuttingAnimationOperation(-1);
        laserAnimationPlaying_ = false;
        if (laserAnimationTimer_) {
            laserAnimationTimer_->stop();
        }
    }

    if (view_) {
        view_->setCuttingAnimationProgress(
            laserAnimationProgress_
        );
    }

    updateLaserAnimationUi();
    refreshAdaptiveRepairView();
}

void MainWindow::updateLaserAnimationUi() {
    const bool hasRoute =
        cuttingRouteCheck_ &&
        cuttingRouteCheck_->isChecked() &&
        !result_.sheets.empty();

    if (laserPlayButton_) {
        laserPlayButton_->setEnabled(
            hasRoute && !laserAnimationPlaying_
        );
    }
    if (laserPauseButton_) {
        laserPauseButton_->setEnabled(
            hasRoute && laserAnimationPlaying_
        );
    }
    if (laserPrevButton_) {
        laserPrevButton_->setEnabled(
            hasRoute && !laserAnimationPlaying_
        );
    }
    if (laserNextButton_) {
        laserNextButton_->setEnabled(
            hasRoute && !laserAnimationPlaying_
        );
    }
    if (laserOperationCombo_) {
        laserOperationCombo_->setEnabled(
            hasRoute && !laserAnimationPlaying_
        );
    }
    const bool hasSelectedContour =
        hasRoute &&
        !laserAnimationPlaying_ &&
        laserAnimationOperation_ >= 0 &&
        static_cast<std::size_t>(laserAnimationOperation_) <
            view_->cuttingRouteOperations().size();
    const auto setContourControl = [hasSelectedContour](QWidget* widget) {
        if (widget) widget->setEnabled(hasSelectedContour);
    };
    setContourControl(laserContourStartButton_);
    setContourControl(laserContourMiddleButton_);
    setContourControl(laserContourEndButton_);
    setContourControl(laserContourStepBackButton_);
    setContourControl(laserContourStepForwardButton_);
    if (laserContourProgressSlider_) {
        laserContourProgressSlider_->setEnabled(hasSelectedContour);
    }
    if (laserResetButton_) {
        laserResetButton_->setEnabled(
            hasRoute
        );
    }
    if (laserSpeedCombo_) {
        laserSpeedCombo_->setEnabled(hasRoute);
    }

    if (!hasRoute) {
        if (laserStageLabel_) {
            laserStageLabel_->setText(
                "Нет готовой раскладки для анимации"
            );
        }
        return;
    }

    const int percent =
        static_cast<int>(
            std::round(
                laserAnimationProgress_ * 100.0
            )
        );

    QString stage;
    if (laserAnimationOperation_ >= 0 &&
        laserContourProgressSlider_) {
        const int contourPercent =
            static_cast<int>(std::lround(laserContourProgress_ * 100.0));
        stage = QString("Операция %1 • контур %2%")
            .arg(laserAnimationOperation_ + 1)
            .arg(contourPercent);
    } else if (percent >= 100) {
        stage = "Готово • маршрут завершён";
    } else if (percent <= 0) {
        stage = "Старт • лазерная головка готова";
    } else if (laserAnimationPlaying_) {
        stage = "Выполнение маршрута";
    } else {
        stage = "Пауза";
    }

    if (laserStageLabel_) {
        laserStageLabel_->setText(
            QString("%1 • %2%")
                .arg(stage)
                .arg(percent)
        );
    }
}

void MainWindow::stopCalculation(bool watchdogTriggered) {
    if (!nestingController_) return;

    if (watchdogTriggered) {
        watchdogTriggered_ = true;
        appendLog("GUI Watchdog: расчёт не отвечает — отправлен запрос отмены.");
        progressDetails_->setText("Watchdog: остановка зависшего расчёта…");
        statusBar()->showMessage("Watchdog: остановка расчёта…");
    } else {
        userCancelRequested_ = true;
        appendLog("Пользователь запросил остановку расчёта.");
        progressDetails_->setText("Остановка расчёта… ожидается завершение worker.");
        statusBar()->showMessage("Остановка расчёта…");
    }

    nestingController_->requestCancel();
    stopButton_->setEnabled(false);
}

void MainWindow::calculationWatchdogTick() {
    if (!watcher_ || !watcher_->isRunning() || !nestingController_) {
        if (calculationWatchdog_) calculationWatchdog_->stop();
        return;
    }

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const qint64 elapsed = now - calculationStartedMs_;
    const qint64 sinceProgress = now - lastProgressMs_;
    const qint64 budgetMs =
        static_cast<qint64>(timeBudgetSpin_->value()) * 1000;

    // The engine owns the normal deadline. The GUI watchdog is a secondary
    // safety net: it never kills a healthy long NFP operation merely because
    // no iteration event arrived yet. It only enforces a hard GUI-side
    // deadline with a grace period, while the UI remains responsive.
    constexpr qint64 kDeadlineGraceMs = 10000;

    if (!watchdogTriggered_ &&
        elapsed >= budgetMs + kDeadlineGraceMs) {
        stopCalculation(true);
        return;
    }

    if (sinceProgress >= 15000) {
        progressDetails_->setText(
            QString("Расчёт продолжается… последний прогресс %1 с назад.")
                .arg(static_cast<qlonglong>(sinceProgress / 1000))
        );
    }

    if (watchdogTriggered_) {
        constexpr qint64 kCancelGraceMs = 10000;
        if (elapsed >= budgetMs + kDeadlineGraceMs + kCancelGraceMs) {
            progressDetails_->setText(
                "Worker завершает отменённый расчёт… "
                "ожидается безопасное завершение."
            );
        }
    }
}

void MainWindow::setBusy(bool busy) {
    importButton_->setEnabled(!busy);
    calculateButton_->setEnabled(!busy && !instances_.empty());
    repairButton_->setEnabled(
        !busy &&
        !instances_.empty() &&
        !result_.sheets.empty() &&
        !validation_.valid
    );
    benchmarkButton_->setEnabled(!busy && !instances_.empty());
    exportButton_->setEnabled(
        !busy &&
        !result_.sheets.empty() &&
        validation_.valid
    );
    benchmarkExportButton_->setEnabled(!busy && hasBenchmarkResult_);

    const bool hasRepairHistory =
        !validation_.adaptiveHistory.empty();
    if (repairPlayButton_) {
        repairPlayButton_->setEnabled(
            !busy &&
            hasRepairHistory &&
            !repairAnimationPlaying_
        );
    }
    if (repairPauseButton_) {
        repairPauseButton_->setEnabled(
            !busy &&
            hasRepairHistory &&
            repairAnimationPlaying_
        );
    }
    if (repairPrevButton_) {
        repairPrevButton_->setEnabled(
            !busy && hasRepairHistory
        );
    }
    if (repairNextButton_) {
        repairNextButton_->setEnabled(
            !busy && hasRepairHistory
        );
    }
    if (repairSpeedCombo_) {
        repairSpeedCombo_->setEnabled(
            !busy && hasRepairHistory
        );
    }

    stopButton_->setEnabled(busy && nestingController_ != nullptr);

    if (laserPlayButton_) {
        laserPlayButton_->setEnabled(
            !busy &&
            cuttingRouteCheck_ &&
            cuttingRouteCheck_->isChecked() &&
            !result_.sheets.empty() &&
            !laserAnimationPlaying_
        );
    }
    if (laserPauseButton_) {
        laserPauseButton_->setEnabled(
            !busy && laserAnimationPlaying_
        );
    }
    if (laserResetButton_) {
        laserResetButton_->setEnabled(
            !busy &&
            cuttingRouteCheck_ &&
            cuttingRouteCheck_->isChecked() &&
            !result_.sheets.empty()
        );
    }
    if (laserSpeedCombo_) {
        laserSpeedCombo_->setEnabled(
            !busy &&
            cuttingRouteCheck_ &&
            cuttingRouteCheck_->isChecked() &&
            !result_.sheets.empty()
        );
    }

    if (busy) {
        progress_->setRange(0, 0);
        statusBar()->showMessage("Выполняется расчёт…");
    } else {
        progress_->setRange(0, 100);
        statusBar()->showMessage("Готово");
    }
}
