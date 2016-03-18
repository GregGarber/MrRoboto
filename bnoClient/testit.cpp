#include "testit.h"

TestIt::TestIt(QObject *parent) : QObject(parent)
{
    client = new QMQTT::Client(EXAMPLE_HOST, EXAMPLE_PORT, this);
    client->setCleanSession(true);
    client->setAutoReconnect(true);
    connect(client, SIGNAL(error(QMQTT::ClientError)), this, SLOT(error(QMQTT::ClientError)));
    connect(client, SIGNAL(connected()), this, SLOT(on_connect()));
    connect(client, SIGNAL(received(QMQTT::Message)), this, SLOT(on_received(QMQTT::Message)));
    client->setClientId("DoogyHowser");
    client->connectToHost();
}

TestIt::~TestIt(){
}

void TestIt::error(const QMQTT::ClientError error){
    qDebug() << "Errro:"<< error;
}

void TestIt::test(){
    emit testme(	);
}

void TestIt::on_connect(){
    qDebug() << "Connected";
    client->subscribe(EXAMPLE_TOPIC,0);
    if(client->isConnectedToHost()){
        QMQTT::Message message(_number, EXAMPLE_TOPIC, QString("how does this work %1").arg(_number).toUtf8());
        _number++;
        client->publish( message);
    }else{
        qDebug() << "no connection " << client->host() << client->port();
    }

}
void TestIt::on_received(QMQTT::Message msg){
    qDebug() << "received:"<<msg.topic()<< " > " <<msg.payload();
}

void TestIt::on_disconnect(){
    qDebug() << "sensing a disconnect";

}

void TestIt::on_publish(){
    qDebug() << "published something";

}
