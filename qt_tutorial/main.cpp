#include "my_window.h"
#include <QApplication>

int main(int argc, char *argv[])
{
#ifdef WIN32
    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setStyle("fusion");
#endif

    QApplication a(argc, argv);
    MyWindow w;
    w.show();

    return a.exec();
}
