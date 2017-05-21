#include "sslConstants.h"
#include "staticObjectsPlugin/staticObjectsConstants.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_refbox_log.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_static_objects.pb.h"

void sslCallback(const std_msgs::String::ConstPtr &msg) {
    SSL_WrapperPacket wp;
    wp.ParseFromString(msg->data.c_str());
    wp.PrintDebugString();
}

void staticObjectsPluginCallback(const std_msgs::String::ConstPtr &msg) {
    StaticObjectsProtobuf::StaticObjects so;
    so.ParseFromString(msg->data.c_str());
    so.PrintDebugString();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, sslConstants::CLIENT_NODE_NAME);
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe(sslConstants::DATA_TOPIC_NAME, 1000, sslCallback);
    ros::Subscriber sub2 = n.subscribe(staticObjectsConstants::DATA_TOPIC_NAME, 1000, staticObjectsPluginCallback);
    ros::spin();

    return 0;
}
