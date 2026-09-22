#include <QApplication>
#include <QThread>
#include <QEventLoop>

#include "splashscreen.hpp"

#include "mainwindow.hpp"

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);

    QApplication::setApplicationName("SheetNest");
    QApplication::setApplicationVersion("0.4.0");
    QApplication::setOrganizationName("SheetNest");

    MainWindow window;

    SplashScreen splash;
    splash.show();
    splash.raise();
    splash.activateWindow();
    window.show();
    window.hide();

    while (!splash.finished()) {
        application.processEvents(
            QEventLoop::AllEvents,
            25
        );
        QThread::msleep(10);
    }

    splash.close();
    window.show();
    window.raise();
    window.activateWindow();

    return application.exec();
}
