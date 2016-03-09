#ifndef NETWORK_H
#define NETWORK_H

#include <QObject>
#include <QDebug>
#include <QTimer>
#include <QtNetwork/QUdpSocket>

class Network : public QObject
{
    Q_OBJECT
public:
    explicit Network(QObject *parent = 0);

signals:
    void  finished();

public slots:
   // void startBroadcasting();
    void broadcastDatagram();

private:
    QUdpSocket *udpSocket;
    QTimer *timer;
    int messageNo;
};

#endif // NETWORK_H
