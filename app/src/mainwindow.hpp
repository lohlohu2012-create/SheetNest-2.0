#pragma once
#include "sheetnest/dxf.hpp"
#include "sheetnest/dxf_export.hpp"
#include "sheetnest/laser_technology.hpp"
#include "sheetnest/nesting.hpp"
#include <QMainWindow>
#include <QPointer>
#include <QTimer>
#include <QFutureWatcher>
#include <vector>

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QLabel;
class QProgressBar;
class QTableWidget;
class NestingView;

class MainWindow final : public QMainWindow {
public:
  explicit MainWindow(QWidget* parent=nullptr);

private:
  void buildUi();
  void openDxf();
  void calculate();
  void exportDxf();
  void openTechnologyEditor();
  void refreshTechnologyChoices();
  void refreshTechnologySelection();
  void showResult();
  void updateSummary(const sheetnest::Result& result);
  QString ensureTechnologyFile();
  std::vector<sheetnest::Polygon> buildCutContours() const;

  NestingView* view_{};
  QComboBox* material_{};
  QComboBox* gas_{};
  QDoubleSpinBox* thickness_{};
  QDoubleSpinBox* sheetWidth_{};
  QDoubleSpinBox* sheetHeight_{};
  QDoubleSpinBox* margin_{};
  QDoubleSpinBox* gap_{};
  QDoubleSpinBox* laserPower_{};
  QDoubleSpinBox* cuttingSpeed_{};
  QDoubleSpinBox* pierceSeconds_{};
  QDoubleSpinBox* rapidSpeed_{};
  QSpinBox* iterations_{};
  QSpinBox* parallelism_{};
  QLabel* fileLabel_{};
  QLabel* summaryLabel_{};
  QLabel* technologyLabel_{};
  QLabel* progressLabel_{};
  QProgressBar* progress_{};
  QTimer progressTimer_;

  std::vector<sheetnest::Instance> instances_;
  sheetnest::Result lastResult_;
  sheetnest::LaserTechnologyDatabase techDb_;
  QString techFilePath_;
  QString currentFile_;
};
