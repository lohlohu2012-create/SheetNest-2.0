#include <QApplication>
#include <QMainWindow>
#include <QDockWidget>
#include <QLabel>
#include <QTreeWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QMenuBar>
#include <QStatusBar>
#include "sheetnest/dxf.hpp"
class MainWindow:public QMainWindow{public:MainWindow(){setWindowTitle("SheetNest 2.0");resize(1500,900);auto *label=new QLabel("SheetNest 2.0\n\nDXF → Nesting → Cutting Path → Time");label->setAlignment(Qt::AlignCenter);setCentralWidget(label);auto *m=menuBar()->addMenu("Проект");auto *open=m->addAction("Импорт DXF...");connect(open,&QAction::triggered,this,[this]{auto f=QFileDialog::getOpenFileName(this,"DXF","", "DXF (*.dxf)");if(!f.isEmpty())statusBar()->showMessage("Загружен: "+f);});statusBar()->showMessage("Готов");}};
int main(int argc,char**argv){QApplication a(argc,argv);MainWindow w;w.show();return a.exec();}
