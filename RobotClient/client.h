#ifndef CLIENT_H
#define CLIENT_H

#include <QObject>
#include <QtCore>
#include <QtNetwork/QTcpSocket>

class QTcpSocket;
class QNetworkSession;

class Client : public QObject
{
    Q_OBJECT
public:
    explicit Client(QObject *parent = 0);

signals:

public slots:
    void requestNewFortune();
    void readFortune();
    void displayError(QAbstractSocket::SocketError socketError);
    void sessionOpened();

private:

    bool isConnected;
    QTcpSocket *tcpSocket;
    QString currentFortune;
    quint16 blockSize;
    QNetworkSession *networkSession;
    QTimer timer;
};

#endif // CLIENT_H
