#include "network.h"

Network::Network(QObject *parent) : QObject(parent)
{
    timer = new QTimer(this);
    udpSocket = new QUdpSocket(this);
    messageNo = 1;
    connect(timer, SIGNAL(timeout()), this, SLOT(broadcastDatagram()));
    timer->start(1000);
}

void Network::broadcastDatagram()
{
    QByteArray datagram = "Broadcast message " + QByteArray::number(messageNo);
    udpSocket->writeDatagram(datagram.data(), datagram.size(),
                             QHostAddress::Broadcast, 45454);
    ++messageNo;
    qDebug() << "Broadcasted";
    emit finished();
}

