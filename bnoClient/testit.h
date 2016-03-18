#ifndef TESTIT_H
#define TESTIT_H

#include <QObject>
#include<QDebug>
#include<qmqtt/qmqtt.h>

const QHostAddress EXAMPLE_HOST = QHostAddress::LocalHost;
const quint16 EXAMPLE_PORT = 1883;
const QString EXAMPLE_TOPIC = "testing";

class TestIt : public QObject
{
    Q_OBJECT
public:
    explicit TestIt(QObject *parent = 0);
    ~TestIt();
    void test();
    void on_disconnect();
    void on_publish();

signals:
    void testme();

public slots:
    void error(const QMQTT::ClientError error);
    void on_connect();
    void on_received(QMQTT::Message);

private:
    QMQTT::Client *client;
    int _number = 1;
};

#endif // TESTIT_H
