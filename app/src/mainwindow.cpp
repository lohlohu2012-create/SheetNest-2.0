#include "mainwindow.hpp"
#include "nesting_view.hpp"
#include <QtConcurrent/QtConcurrentRun>
#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QAction>
#include <QDialog>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSettings>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTableWidget>
#include <QToolBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStandardPaths>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <memory>

namespace {
QDoubleSpinBox* makeDouble(double value,double min,double max,double step,QWidget* parent) {
  auto* s=new QDoubleSpinBox(parent);
  s->setRange(min,max); s->setSingleStep(step); s->setValue(value); s->setDecimals(3);
  return s;
}
QSpinBox* makeInt(int value,int min,int max,QWidget* parent) {
  auto* s=new QSpinBox(parent); s->setRange(min,max); s->setValue(value); return s;
}
QString fmt(double v,int digits=2) { return QString::number(v,'f',digits); }
}

MainWindow::MainWindow(QWidget* parent):QMainWindow(parent) {
  techFilePath_=ensureTechnologyFile();
  techDb_.loadCsv(techFilePath_.toStdString());
  buildUi();
  refreshTechnologyChoices();

  connect(material_,&QComboBox::currentTextChanged,this,[this]{ refreshTechnologySelection(); });
  connect(gas_,&QComboBox::currentTextChanged,this,[this]{ refreshTechnologySelection(); });
  connect(thickness_,qOverload<double>(&QDoubleSpinBox::valueChanged),this,[this]{ refreshTechnologySelection(); });

  setWindowTitle("SheetNest 2.0 — Metal Sheet Nesting");
  resize(1600,950);
  statusBar()->showMessage("Готово");
}

QString MainWindow::ensureTechnologyFile() {
  const QString external=QCoreApplication::applicationDirPath()+"/laser_bodor_3kw.csv";
  if(QFile::exists(external)) return external;
  if(QFile::copy(":/sheetnest/laser_bodor_3kw.csv",external)) return external;

  const QString fallbackDir=QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
  QDir().mkpath(fallbackDir);
  const QString fallback=fallbackDir+"/laser_bodor_3kw.csv";
  if(!QFile::exists(fallback)) QFile::copy(":/sheetnest/laser_bodor_3kw.csv",fallback);
  return fallback;
}

void MainWindow::buildUi() {
  setStyleSheet(R"(
    QMainWindow { background:#11161a; color:#e7edf2; }
    QDockWidget { color:#e7edf2; }
    QWidget#Panel { background:#1a2127; border:1px solid #2b353d; }
    QLabel#Title { font-size:18px; font-weight:700; color:#f1f4f6; }
    QLabel#SubTitle { color:#8e9ba6; }
    QPushButton { background:#26323a; border:1px solid #3b4851; border-radius:5px; padding:8px 12px; }
    QPushButton:hover { background:#30404a; }
    QPushButton#Primary { background:#16786f; border-color:#25a499; font-weight:700; }
    QDoubleSpinBox,QSpinBox,QComboBox,QLineEdit {
      background:#10151a; border:1px solid #38444d; border-radius:4px; padding:5px; color:#e7edf2;
    }
    QTableWidget { background:#10151a; gridline-color:#2c363d; }
    QHeaderView::section { background:#202930; color:#dce4e8; padding:5px; border:0; }
    QProgressBar { background:#10151a; border:1px solid #36424a; height:10px; border-radius:4px; }
    QProgressBar::chunk { background:#1ca394; border-radius:4px; }
  )");

  auto* toolbar=addToolBar("Основные операции");
  toolbar->setMovable(false);

  auto* openAct=new QAction("Открыть DXF",this);
  auto* calcAct=new QAction("Рассчитать",this);
  auto* exportAct=new QAction("Экспорт DXF",this);
  auto* techAct=new QAction("Технология лазера 3 кВт",this);
  toolbar->addAction(openAct);
  toolbar->addAction(calcAct);
  toolbar->addAction(exportAct);
  toolbar->addSeparator();
  toolbar->addAction(techAct);

  connect(openAct,&QAction::triggered,this,[this]{openDxf();});
  connect(calcAct,&QAction::triggered,this,[this]{calculate();});
  connect(exportAct,&QAction::triggered,this,[this]{exportDxf();});
  connect(techAct,&QAction::triggered,this,[this]{openTechnologyEditor();});

  auto* central=new QWidget(this);
  auto* centralLayout=new QVBoxLayout(central);
  centralLayout->setContentsMargins(6,6,6,6);
  view_=new NestingView(central);
  centralLayout->addWidget(view_,1);

  auto* progressRow=new QHBoxLayout;
  progressLabel_=new QLabel("Готово",central);
  progress_=new QProgressBar(central);
  progress_->setRange(0,1); progress_->setValue(1);
  progressRow->addWidget(progressLabel_);
  progressRow->addWidget(progress_,1);
  centralLayout->addLayout(progressRow);
  setCentralWidget(central);

  auto* params=new QDockWidget("Параметры раскроя",this);
  params->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);
  auto* panel=new QWidget(params); panel->setObjectName("Panel");
  auto* form=new QFormLayout(panel);
  form->setContentsMargins(12,12,12,12);

  fileLabel_=new QLabel("DXF не загружен",panel);
  fileLabel_->setWordWrap(true);
  form->addRow("Файл:",fileLabel_);

  sheetWidth_=makeDouble(1500,10,100000,10,panel);
  sheetHeight_=makeDouble(3000,10,100000,10,panel);
  margin_=makeDouble(5,0,100,0.5,panel);
  gap_=makeDouble(2,0,50,0.1,panel);
  iterations_=makeInt(64,1,256,panel);
  parallelism_=makeInt(0,0,128,panel);
  laserPower_=makeDouble(3,0.1,100,0.5,panel);
  thickness_=makeDouble(3,0.1,100,0.1,panel);
  cuttingSpeed_=makeDouble(1,0.1,500,0.1,panel);
  pierceSeconds_=makeDouble(0.25,0.01,30,0.01,panel);
  rapidSpeed_=makeDouble(120,1,500,5,panel);
  material_=new QComboBox(panel);
  gas_=new QComboBox(panel);
  technologyLabel_=new QLabel("Нет выбранного режима",panel);
  technologyLabel_->setWordWrap(true);

  form->addRow("Ширина листа, мм:",sheetWidth_);
  form->addRow("Высота листа, мм:",sheetHeight_);
  form->addRow("Кромка, мм:",margin_);
  form->addRow("Зазор, мм:",gap_);
  form->addRow("Материал:",material_);
  form->addRow("Толщина, мм:",thickness_);
  form->addRow("Газ:",gas_);
  form->addRow("Мощность, кВт:",laserPower_);
  form->addRow("Скорость реза, м/мин:",cuttingSpeed_);
  form->addRow("Прокол, сек:",pierceSeconds_);
  form->addRow("Rapid, м/мин:",rapidSpeed_);
  form->addRow("Стартов поиска:",iterations_);
  form->addRow("Параллельные потоки:",parallelism_);
  form->addRow("Технология:",technologyLabel_);

  auto* calcButton=new QPushButton("РАССЧИТАТЬ РАСКРОЙ",panel);
  calcButton->setObjectName("Primary");
  auto* techButton=new QPushButton("Открыть базу технологий",panel);
  form->addRow(calcButton);
  form->addRow(techButton);
  connect(calcButton,&QPushButton::clicked,this,[this]{calculate();});
  connect(techButton,&QPushButton::clicked,this,[this]{openTechnologyEditor();});

  params->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea,params);

  auto* left=new QDockWidget("Проект",this);
  auto* lp=new QWidget(left); lp->setObjectName("Panel");
  auto* l=new QVBoxLayout(lp);
  auto* title=new QLabel("SheetNest 2.0",lp); title->setObjectName("Title");
  auto* subtitle=new QLabel("2D раскрой металлического листа",lp); subtitle->setObjectName("SubTitle");
  summaryLabel_=new QLabel("Деталей: 0",lp);
  summaryLabel_->setWordWrap(true);
  l->addWidget(title); l->addWidget(subtitle); l->addSpacing(12);
  auto* b1=new QPushButton("Открыть DXF",lp);
  auto* b2=new QPushButton("Рассчитать",lp); b2->setObjectName("Primary");
  auto* b3=new QPushButton("Экспорт DXF",lp);
  l->addWidget(b1); l->addWidget(b2); l->addWidget(b3);
  l->addSpacing(16);
  l->addWidget(summaryLabel_);
  l->addStretch(1);
  left->setWidget(lp);
  addDockWidget(Qt::LeftDockWidgetArea,left);
  connect(b1,&QPushButton::clicked,this,[this]{openDxf();});
  connect(b2,&QPushButton::clicked,this,[this]{calculate();});
  connect(b3,&QPushButton::clicked,this,[this]{exportDxf();});

  connect(&progressTimer_,&QTimer::timeout,this,[this]{
    static int dots=0; dots=(dots+1)%4;
    progressLabel_->setText(QString("Расчёт") + QString(".").repeated(dots));
  });
}

void MainWindow::refreshTechnologyChoices() {
  material_->blockSignals(true);
  gas_->blockSignals(true);
  material_->clear();
  gas_->clear();

  std::vector<QString> materials,gases;
  for(const auto& p:techDb_.points()) {
    if(std::find(materials.begin(),materials.end(),QString::fromStdString(p.material))==materials.end())
      materials.push_back(QString::fromStdString(p.material));
    if(std::find(gases.begin(),gases.end(),QString::fromStdString(p.gas))==gases.end())
      gases.push_back(QString::fromStdString(p.gas));
  }
  for(const auto& m:materials)material_->addItem(m);
  for(const auto& g:gases)gas_->addItem(g);

  const int cs=material_->findText("Carbon Steel");
  if(cs>=0)material_->setCurrentIndex(cs);
  const int o2=gas_->findText("O2");
  if(o2>=0)gas_->setCurrentIndex(o2);

  material_->blockSignals(false);
  gas_->blockSignals(false);
  refreshTechnologySelection();
}

void MainWindow::refreshTechnologySelection() {
  if(!material_||!gas_||!thickness_)return;
  const auto tech=techDb_.lookup(material_->currentText().toStdString(),thickness_->value(),
                                 gas_->currentText().toStdString(),laserPower_->value());
  if(!tech) {
    technologyLabel_->setText("Нет точки в базе — скорость задаётся вручную.");
    return;
  }
  cuttingSpeed_->setValue(tech->speedMMin);
  pierceSeconds_->setValue(tech->pierceSeconds);
  technologyLabel_->setText(
    QString("%1–%2 м/мин; %3 сек/прокол%4")
      .arg(fmt(tech->speedMinMMin)).arg(fmt(tech->speedMaxMMin))
      .arg(fmt(tech->pierceSeconds,3))
      .arg(tech->interpolated?" · интерполяция":""));
}

void MainWindow::openDxf() {
  const QString path=QFileDialog::getOpenFileName(this,"Открыть DXF",{}, "DXF (*.dxf)");
  if(path.isEmpty())return;

  const auto doc=sheetnest::importDxfFile(path.toStdString(),0.25);
  if(doc.contours.empty()) {
    QString msg="DXF не содержит валидных замкнутых контуров.";
    if(!doc.diagnostics.empty())msg+="\n"+QString::fromStdString(doc.diagnostics.front().message);
    QMessageBox::warning(this,"Ошибка DXF",msg);
    return;
  }

  instances_.clear();
  int index=1;
  for(const auto& contour:doc.contours) {
    sheetnest::Part part;
    part.id="DXF_PART_"+std::to_string(index);
    part.shape={contour.outer,contour.holes};

    sheetnest::Instance inst;
    inst.id="instance-"+std::to_string(index);
    inst.unitId="unit-"+std::to_string(index);
    inst.part=std::move(part);
    instances_.push_back(std::move(inst));
    ++index;
  }

  currentFile_=path;
  fileLabel_->setText(QFileInfo(path).fileName());
  summaryLabel_->setText(QString("Деталей: %1\nКонтуров: %2\nОшибок/предупреждений: %3")
    .arg(instances_.size()).arg(doc.contours.size()).arg(doc.diagnostics.size()));
  statusBar()->showMessage(QString("Загружено: %1").arg(QFileInfo(path).fileName()));
  view_->clearLayout();
}

void MainWindow::calculate() {
  if(instances_.empty()) {
    QMessageBox::information(this,"SheetNest","Сначала загрузите DXF.");
    return;
  }

  sheetnest::Options opt;
  opt.iterations=iterations_->value();
  opt.parallelism=parallelism_->value();
  opt.gapMm=gap_->value();
  opt.candidateGridMm=5.0;
  opt.sheetReductionPasses=4;

  const sheetnest::Sheet sheet{sheetWidth_->value(),sheetHeight_->value(),margin_->value()};
  const auto parts=instances_;

  progress_->setRange(0,0);
  progressLabel_->setText("Подготовка...");
  progressTimer_.start(350);
  setEnabledForCalculation(false);

  auto* watcher=new QFutureWatcher<sheetnest::Result>(this);
  watcher->setFuture(QtConcurrent::run([parts,sheet,opt]{
    return sheetnest::nest(parts,sheet,opt);
  }));

  connect(watcher,&QFutureWatcher<sheetnest::Result>::finished,this,[this,watcher]{
    lastResult_=watcher->result();
    watcher->deleteLater();

    progressTimer_.stop();
    progress_->setRange(0,1);
    progress_->setValue(1);
    progressLabel_->setText(lastResult_.complete()?"Расчёт завершён":"Расчёт завершён с потерями");

    showResult();
    setEnabledForCalculation(true);
  });
}

void MainWindow::setEnabledForCalculation(bool enabled) {
  statusBar()->showMessage(enabled?"Готово":"Идёт параллельный nesting...");
  material_->setEnabled(enabled);
  gas_->setEnabled(enabled);
  thickness_->setEnabled(enabled);
  sheetWidth_->setEnabled(enabled);
  sheetHeight_->setEnabled(enabled);
  margin_->setEnabled(enabled);
  gap_->setEnabled(enabled);
  laserPower_->setEnabled(enabled);
  cuttingSpeed_->setEnabled(enabled);
  pierceSeconds_->setEnabled(enabled);
  rapidSpeed_->setEnabled(enabled);
  iterations_->setEnabled(enabled);
  parallelism_->setEnabled(enabled);
}

std::vector<sheetnest::Polygon> MainWindow::buildCutContours() const {
  std::vector<sheetnest::Polygon> contours;
  std::unordered_map<std::string,const sheetnest::Instance*> byId;
  for(const auto& i:instances_)byId[i.id]=&i;

  auto oriented=[](sheetnest::Polygon p,bool ccw){
    if(p.size()<3)return p;
    const bool positive=sheetnest::signedPolygonArea(p)>0;
    if(positive!=ccw)std::reverse(p.begin(),p.end());
    return p;
  };

  for(const auto& sheet:lastResult_.sheets) {
    for(const auto& placement:sheet) {
      auto it=byId.find(placement.id);
      if(it==byId.end())continue;
      auto shape=sheetnest::normalized(sheetnest::rotate(it->second->part.shape,placement.rotation));
      shape=sheetnest::translate(shape,placement.x,placement.y);
      contours.push_back(oriented(shape.outer,true));
      for(auto hole:shape.holes)contours.push_back(oriented(std::move(hole),false));
    }
  }
  return contours;
}

void MainWindow::showResult() {
  view_->setResult(lastResult_,instances_);
  updateSummary(lastResult_);

  const auto contours=buildCutContours();
  const sheetnest::CuttingParameters cp{
    laserPower_->value(),
    material_->currentText().toStdString(),
    thickness_->value(),
    gas_->currentText().toStdString(),
    cuttingSpeed_->value(),
    pierceSeconds_->value()
  };
  const sheetnest::PathOptions po{rapidSpeed_->value(),pierceSeconds_->value()};
  const auto path=sheetnest::planCuttingPath(contours,cp,po);
  const auto estimate=sheetnest::estimateCuttingPath(path,cp,po);

  summaryLabel_->setText(summaryLabel_->text()
    + QString("\n\nДлина реза: %1 мм\nПроколов: %2\nRapid: %3 мм\nВремя резки: %4 мин")
        .arg(fmt(estimate.contourLengthMm,1))
        .arg(estimate.pierces)
        .arg(fmt(path.totalRapidLengthMm,1))
        .arg(fmt(estimate.totalMinutes,2)));
}

void MainWindow::updateSummary(const sheetnest::Result& result) {
  std::size_t placed=0;
  for(const auto& s:result.sheets)placed+=s.size();
  summaryLabel_->setText(QString("Листов: %1\nРазмещено: %2\nПотеряно: %3\nЗаполнение: %4%\nИтераций: %5")
    .arg(result.sheets.size())
    .arg(placed)
    .arg(result.unplaced.size())
    .arg(fmt(result.utilization*100.0,2))
    .arg(result.iterations));

  if(!result.diagnostics.empty()) {
    QStringList lines;
    for(const auto& d:result.diagnostics)
      lines<<QString("%1: %2").arg(QString::fromStdString(d.instanceId))
        .arg(QString::fromStdString(d.stage));
    QMessageBox::information(this,"Диагностика nesting",
      "Потерянные детали:\n"+lines.join("\n"));
  }
}

void MainWindow::exportDxf() {
  if(lastResult_.sheets.empty()) {
    QMessageBox::information(this,"Экспорт","Сначала выполните nesting.");
    return;
  }
  const QString path=QFileDialog::getSaveFileName(this,"Экспорт раскладки DXF",{}, "DXF (*.dxf)");
  if(path.isEmpty())return;
  if(!sheetnest::exportNestDxfFile(path.toStdString(),instances_,lastResult_))
    QMessageBox::warning(this,"Экспорт","Не удалось записать DXF.");
  else
    statusBar()->showMessage("DXF экспортирован.");
}

void MainWindow::openTechnologyEditor() {
  QDialog dialog(this);
  dialog.setWindowTitle("Технология лазера 3 кВт");
  dialog.resize(1050,650);
  auto* layout=new QVBoxLayout(&dialog);

  auto* info=new QLabel(
    "Стартовая база. Диапазоны и скорости редактируются вручную; перед производственной работой "
    "проверьте их на конкретных источнике, голове, сопле, газе и материале.",
    &dialog);
  info->setWordWrap(true);
  layout->addWidget(info);

  auto* table=new QTableWidget(&dialog);
  table->setColumnCount(8);
  table->setHorizontalHeaderLabels({"Материал","Толщина","Газ","Мин","Макс","Скорость","Прокол","Источник"});
  table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  table->setRowCount(int(techDb_.points().size()));

  for(int r=0;r<int(techDb_.points().size());++r) {
    const auto& p=techDb_.points()[r];
    table->setItem(r,0,new QTableWidgetItem(QString::fromStdString(p.material)));
    table->setItem(r,1,new QTableWidgetItem(QString::number(p.thicknessMm)));
    table->setItem(r,2,new QTableWidgetItem(QString::fromStdString(p.gas)));
    table->setItem(r,3,new QTableWidgetItem(QString::number(p.speedMinMMin)));
    table->setItem(r,4,new QTableWidgetItem(QString::number(p.speedMaxMMin)));
    table->setItem(r,5,new QTableWidgetItem(QString::number(p.speedMMin)));
    table->setItem(r,6,new QTableWidgetItem(QString::number(p.pierceSeconds)));
    table->setItem(r,7,new QTableWidgetItem(QString::fromStdString(p.source)));
  }
  layout->addWidget(table,1);

  auto* buttons=new QHBoxLayout;
  auto* save=new QPushButton("Сохранить", &dialog);
  auto* cancel=new QPushButton("Отмена", &dialog);
  buttons->addStretch(1); buttons->addWidget(save); buttons->addWidget(cancel);
  layout->addLayout(buttons);

  connect(cancel,&QPushButton::clicked,&dialog,&QDialog::reject);
  connect(save,&QPushButton::clicked,&dialog,[this,&dialog,table]{
    auto points=techDb_.points();
    for(int r=0;r<table->rowCount()&&r<int(points.size());++r) {
      points[r].speedMMin=table->item(r,5)->text().toDouble();
      points[r].pierceSeconds=table->item(r,6)->text().toDouble();
    }
    techDb_.setPoints(std::move(points));
    if(!techDb_.saveCsv(techFilePath_.toStdString())) {
      QMessageBox::warning(&dialog,"Технология","Не удалось сохранить CSV.");
      return;
    }
    refreshTechnologySelection();
    dialog.accept();
  });

  dialog.exec();
}

int main(int argc,char** argv);
