#ifndef SSL_VISION_STATICOBJECTSCONSTANTS_H
#define SSL_VISION_STATICOBJECTSCONSTANTS_H

#include <QtGlobal>
#include <QString>

namespace staticObjectsConstants {
    static const char* DATA_TOPIC_NAME = "staticObjects";
    static const quint16 PORT1 = 10007;
    static const quint16 PORT2 = 10008;
    static const quint16 PORT3 = 10009;
    static const quint16 PORT4 = 10010;
    static const QString LOOPBACK = "127.0.0.1";
    static const char* SERVER_NODE_NAME = "staticObjectsServer";
    static const char* CLIENT_NODE_NAME = "sslObjectClient";
}

#endif //SSL_VISION_STATICOBJECTSCONSTANTS_H
