#include <QtCore>
#include <QtNetwork/QtNetwork>
#include "client.h"

Client::Client(QObject *parent) : QObject(parent), networkSession(0)
{

    tcpSocket = new QTcpSocket(this);
    isConnected = false;

    connect(tcpSocket, SIGNAL(readyRead()), this, SLOT(readFortune()));
    connect(tcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(displayError(QAbstractSocket::SocketError)));

    QNetworkConfigurationManager manager;
    if (manager.capabilities() & QNetworkConfigurationManager::NetworkSessionRequired) {
        // Get saved network configuration
        QSettings settings(QSettings::UserScope, QLatin1String("QtProject"));
        settings.beginGroup(QLatin1String("QtNetwork"));
        const QString id = settings.value(QLatin1String("DefaultNetworkConfiguration")).toString();
        settings.endGroup();

        // If the saved network configuration is not currently discovered use the system default
        QNetworkConfiguration config = manager.configurationFromIdentifier(id);
        if ((config.state() & QNetworkConfiguration::Discovered) !=
            QNetworkConfiguration::Discovered) {
            config = manager.defaultConfiguration();
        }

        networkSession = new QNetworkSession(config, this);
        connect(networkSession, SIGNAL(opened()), this, SLOT(sessionOpened()));

        qDebug() << tr("Opening network session.");
        networkSession->open();
    }

        connect(&timer, SIGNAL(timeout()), this, SLOT(requestNewFortune()));
        timer.setInterval(1000);
        timer.start(1000);
}

void Client::requestNewFortune()
{
        qDebug() << "Sending request for new forturne";
    blockSize = 0;
    tcpSocket->abort();
    tcpSocket->connectToHost(QHostAddress::LocalHost, 45449);
    isConnected = true;
}

/*
void Client::requestNewFortune()
{
    blockSize = 0;
    if(!isConnected){
        tcpSocket->abort();
        tcpSocket->connectToHost(QHostAddress::LocalHost, 45449);
        isConnected = true;
    }

    QString message("This is from the client");
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_4_0);
    out << (quint16)0;
    out << message;
    out.device()->seek(0);
    out << (quint16)(block.size() - sizeof(quint16));

    tcpSocket->write(block);
    //tcpSocket->disconnectFromHost();
}
*/

void Client::readFortune()
{
    qDebug() << "Read Fortune";
    QDataStream in(tcpSocket);
    in.setVersion(QDataStream::Qt_4_0);

    if (blockSize == 0) {
        if (tcpSocket->bytesAvailable() < (int)sizeof(quint16))
            return;

        in >> blockSize;
    }

    if (tcpSocket->bytesAvailable() < blockSize)
        return;

    QString nextFortune;
    in >> nextFortune;

    if (nextFortune == currentFortune) {
        QTimer::singleShot(0, this, SLOT(requestNewFortune()));
        return;
    }

    currentFortune = nextFortune;
    qDebug() << currentFortune;

    qDebug() << "Sending msg to server";
    QString message("This is from the client");
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_4_0);
    out << (quint16)0;
    out << message;
    out.device()->seek(0);
    out << (quint16)(block.size() - sizeof(quint16));

    tcpSocket->write(block);
    //tcpSocket->disconnectFromHost();
}

void Client::displayError(QAbstractSocket::SocketError socketError)
{
    switch (socketError) {
    case QAbstractSocket::RemoteHostClosedError:
        break;
    case QAbstractSocket::HostNotFoundError:
        qDebug() << tr("Fortune Client") + tr("The host was not found. Please check the "
                                    "host name and port settings.");
        break;
    case QAbstractSocket::ConnectionRefusedError:
        qDebug() << tr("Fortune Client")+
                                 tr("The connection was refused by the peer. "
                                    "Make sure the fortune server is running, "
                                    "and check that the host name and port "
                                    "settings are correct.");
        break;
    default:
       qDebug() << tr("Fortune Client")+
                                 tr("The following error occurred: %1.").arg(tcpSocket->errorString());
    }

}


void Client::sessionOpened()
{
    // Save the used configuration
    QNetworkConfiguration config = networkSession->configuration();
    QString id;
    if (config.type() == QNetworkConfiguration::UserChoice)
        id = networkSession->sessionProperty(QLatin1String("UserChoiceConfiguration")).toString();
    else
        id = config.identifier();

    QSettings settings(QSettings::UserScope, QLatin1String("QtProject"));
    settings.beginGroup(QLatin1String("QtNetwork"));
    settings.setValue(QLatin1String("DefaultNetworkConfiguration"), id);
    settings.endGroup();

    qDebug() << tr("This examples requires that you run the Fortune Server example as well.");

}
