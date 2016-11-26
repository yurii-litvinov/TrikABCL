#include <QtCore/qglobal.h>

#include <QtCore/QScopedPointer>
#include "distencoder.h"
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    #include <QtGui/QApplication>
#else
    #include <QtWidgets/QApplication>
#endif

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    DistEncoder * dist = new DistEncoder(4,10,10205);
    dist->startMove(100);
    return app.exec();
}
