#include <QCoreApplication>
//#include <QRemoteObjectNode>
#include "bno055.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    BNO055 fusion;
    fusion.init();
    fusion.readCalibrationFile("./calibration");

    /* would have been kind of nice if it had worked, but even repc seemed screwy
    QRemoteObjectRegistryHost regNode(QUrl(QStringLiteral("local:registry"))); // create node that hosts registy
    // create node that will host source and connect to registry
    QRemoteObjectHost srcNode(QUrl(QStringLiteral("local:replica")), QUrl(QStringLiteral("local:registry")));

    //Note, you can add srcSwitch directly to regNode if desired.
    //We use two Nodes here, as the regNode could easily be in a third process.

    srcNode.enableRemoting(&fusion); // enable remoting of source object
    */
    return a.exec();
}

