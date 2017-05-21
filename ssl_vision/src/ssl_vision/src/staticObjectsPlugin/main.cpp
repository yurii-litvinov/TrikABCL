#include <QtCore/qglobal.h>
#include <QObject>
#include <QtNetwork>
#include <QtWidgets/QApplication>

#include "staticObjectsServer.h"
#include "staticObjectsConstants.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    try {
        ros::init(argc, argv, staticObjectsConstants::SERVER_NODE_NAME);
        ros::NodeHandle n;
        ros::Publisher publisher =
                n.advertise<std_msgs::String>(staticObjectsConstants::DATA_TOPIC_NAME, 1000);

        QApplication app(argc, argv);

        QScopedPointer<StaticObjectsServer> s1(
                new StaticObjectsServer(staticObjectsConstants::PORT1, staticObjectsConstants::LOOPBACK, publisher));
        QScopedPointer<StaticObjectsServer> s2(
                new StaticObjectsServer(staticObjectsConstants::PORT2, staticObjectsConstants::LOOPBACK, publisher));
        QScopedPointer<StaticObjectsServer> s3(
                new StaticObjectsServer(staticObjectsConstants::PORT3, staticObjectsConstants::LOOPBACK, publisher));
        QScopedPointer<StaticObjectsServer> s4(
                new StaticObjectsServer(staticObjectsConstants::PORT4, staticObjectsConstants::LOOPBACK, publisher));

        return app.exec();
    } catch (std::runtime_error &e) {
        std::cout << e.what() << "\n";
        return 1;
    }
}
