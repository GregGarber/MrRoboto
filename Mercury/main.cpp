#include <QCoreApplication>
#include <QtCore>
#include <QDebug>
#include <QTimer>
#include <stdlib.h>
//#include "network.h"
#include "server.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    //Network n(&a);
    Server server;
    QObject::connect(&server, SIGNAL(finished()), &a, SLOT(quit()));
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));
    return a.exec();
}

