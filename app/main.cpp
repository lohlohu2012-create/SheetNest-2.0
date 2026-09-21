#include <QApplication>

#include "mainwindow.hpp"

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);

    QApplication::setApplicationName("SheetNest");
    QApplication::setApplicationVersion("0.4.0");
    QApplication::setOrganizationName("SheetNest");

    MainWindow window;
    window.show();

    return application.exec();
}
