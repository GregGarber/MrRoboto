#include <QCoreApplication>
#include "testit.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    TestIt mosq;
    return a.exec();
}
