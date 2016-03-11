#include <QCoreApplication>
#include "bno055.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    BNO055 fusion;
    fusion.init();
    fusion.readCalibrationFile("./calibration");
    fusion.printReadings(10);
    //fusion.printSelfTest();
    //fusion.printCalibrationStatus();
    //fusion.writeCalibrationFile("./calibration");
    return a.exec();
}

