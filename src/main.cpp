#include "MainWindow.h"

#include <QApplication>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("FissionReactorOptimizer");
    QApplication::setApplicationDisplayName(QString::fromUtf8("裂变反应堆搭建优化器"));

    MainWindow window;
    window.show();
    return app.exec();
}
