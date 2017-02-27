#ifndef SSL_VISION_STATICOBJECTSSERVER_H
#define SSL_VISION_STATICOBJECTSSERVER_H

#include <stdexcept>
#include <QObject>
#include <QtNetwork>
#include <QtCore>
#include <QDataStream>

#include "ros/ros.h"
#include "std_msgs/String.h"

class StaticObjectsServer : public QObject {
    Q_OBJECT

public:
    StaticObjectsServer(quint16 port, QString ip, ros::Publisher publisher);
    ~StaticObjectsServer();

private slots:
    void read();
    void disconnected();

private:
    const quint16 mPort;
    const QString mIp;
    QScopedPointer<QTcpSocket> mSocket;
    quint16 mBytesExpected;
    ros::Publisher mPublisher;
};

#endif //SSL_VISION_STATICOBJECTSSERVER_H
