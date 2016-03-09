#ifndef SERVER_H
#define SERVER_H

#include <QObject>
#include <QtCore>

class QTcpServer;
class QTcpSocket;
class QNetworkSession;

class Server : public QObject
{
    Q_OBJECT
public:
    explicit Server(QObject *parent = 0);
    ~Server();

signals:
    void  finished();

public slots:
    void sessionOpened();
    void sendFortune();
    void readMessage();

private:
    quint16 blockSize;
    QTcpServer *tcpServer;
    QTcpSocket *clientConnection;
    QStringList fortunes;
    QNetworkSession *networkSession;
};

#endif // SERVER_H
