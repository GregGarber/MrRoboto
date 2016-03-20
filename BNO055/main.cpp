#include <QCoreApplication>
//#include <QRemoteObjectNode>
#include "bno055.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    BNO055 fusion;
    fusion.init();
    fusion.readCalibrationFile("./calibration");
    return a.exec();
}

