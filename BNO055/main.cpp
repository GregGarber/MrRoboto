#include <QCoreApplication>
#include "bno055.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    BNO055 fusion;

    return a.exec();
}

