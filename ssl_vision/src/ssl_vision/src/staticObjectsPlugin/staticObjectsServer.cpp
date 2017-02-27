#include "staticObjectsServer.h"

StaticObjectsServer::StaticObjectsServer(quint16 port, QString ip, ros::Publisher publisher)
        : QObject(), mPort(port), mIp(ip), mPublisher(publisher), mSocket(new QTcpSocket()) {

    mBytesExpected = 0;

    connect(&*mSocket, SIGNAL(readyRead()), this, SLOT(read()));
    connect(&*mSocket, SIGNAL(disconnected()), this, SLOT(disconnected()));

    mSocket->connectToHost(mIp, mPort, QIODevice::ReadOnly);
    if (!mSocket->waitForConnected()) {
        throw std::runtime_error("Cannot connect to host");
    }
}

StaticObjectsServer::~StaticObjectsServer() {
}

void StaticObjectsServer::read() {
    QDataStream in(&*mSocket);
    in.setVersion(QDataStream::Qt_4_0);

    if (mBytesExpected == 0) {
        if (mSocket->bytesAvailable() < (int)sizeof(quint16)) {
            return;
        }

        in >> mBytesExpected;
    }

    if (mSocket->bytesAvailable() < mBytesExpected) {
        return;
    }

    char *buffer = new char[mBytesExpected];
    uint length = mBytesExpected;
    in.readBytes(buffer, length);

    std_msgs::String msg;
    std::stringstream ss;
    ss << buffer;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    mPublisher.publish(msg);
    ros::spinOnce();

    delete[] buffer;
    mBytesExpected = 0;
}

void StaticObjectsServer::disconnected() {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Port " << mPort << " disconnected";
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
}
