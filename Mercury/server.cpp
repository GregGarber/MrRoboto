#include <QtNetwork>

#include <stdlib.h>

#include "server.h"

Server::Server(QObject *parent) : QObject(parent), tcpServer(0), networkSession(0)
{
    blockSize = 0;
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

        qDebug() << (tr("Opening network session."));
        networkSession->open();
    } else {
        sessionOpened();
    }

        fortunes << tr("You've been leading a dog's life. Stay off the furniture.")
                 << tr("You've got to think about tomorrow.")
                 << tr("You will be surprised by a loud noise.")
                 << tr("You will feel hungry again in another hour.")
                 << tr("You might have mail.")
                 << tr("You cannot kill time without injuring eternity.")
                 << tr("Computers are not intelligent. They only think they are.");

        connect(tcpServer, SIGNAL(newConnection()), this, SLOT(sendFortune()));
}

Server::~Server(){
    tcpServer->close();
}

void Server::sessionOpened()
{
    // Save the used configuration
    if (networkSession) {
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
    }

    tcpServer = new QTcpServer(this);
    if (!tcpServer->listen(QHostAddress::LocalHost, 45449)) {
        tcpServer->deleteLater();
        qDebug() << tr("Fortune Server") << tr("Unable to start the server: %1.").arg(tcpServer->errorString());
        emit finished();
        return;
    }
    qDebug() << "Address:" << tcpServer->serverAddress();

}

void Server::sendFortune()
{
    QByteArray block;
    QDataStream out(&block, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_4_0);
    out << (quint16)0;
    out << fortunes.at(qrand() % fortunes.size());
    out.device()->seek(0);
    out << (quint16)(block.size() - sizeof(quint16));

    clientConnection = tcpServer->nextPendingConnection();
    connect(clientConnection, SIGNAL(disconnected()),
            clientConnection, SLOT(deleteLater()));
    connect(clientConnection, SIGNAL(readyRead()), this, SLOT(readMessage()));

    clientConnection->write(block);
}

void Server::readMessage()
{
    qDebug()<<"read";
    blockSize = 0;
    QDataStream in(clientConnection);
    in.setVersion(QDataStream::Qt_4_0);

    if (blockSize == 0) {
        if (clientConnection->bytesAvailable() < (int)sizeof(quint16)){
            qDebug() << "No bytes available";
            return;
        }
        in >> blockSize;
        qDebug()<< "Block Size is:"<<blockSize;
    }

    if (clientConnection->bytesAvailable() < blockSize){
        qDebug() << "bytes available is less than block size";
        return;
    }
    qDebug() << "Bytes available:"<<clientConnection->bytesAvailable();

    QString message;
    //clientConnection->disconnectFromHost();

   // while(clientConnection->bytesAvailable()){
        in >> message;//this suddenly started working, no explanation
        qDebug() <<"READ "<< message;
    //}
        //idea: look into overriding qObject::connect() so that it can connect
        //to methods over TCP. If is local, default to normal connect()
}
