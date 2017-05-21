#include <netdb.h>
#include <stdexcept>

#include "sslConstants.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

// Returns socket descriptor to which SSL-Vision data is sent.
int getSslDescriptor() {
    const char *hostname = 0;
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = 0;
    hints.ai_flags = AI_PASSIVE | AI_ADDRCONFIG;
    struct addrinfo *res = 0;
    int err = getaddrinfo(hostname, sslConstants::PORT, &hints, &res);
    if (err != 0) {
        throw std::runtime_error(std::string(
                "failed to resolve local socket address (err= ") + std::to_string(err));
    }

    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd == -1) {
        throw std::runtime_error(std::string(strerror(errno)));
    }

    int enable = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        throw std::runtime_error(std::string("setsockopt(SO_REUSEADDR) failed"));
    }

    if (bind(fd, res->ai_addr, res->ai_addrlen) == -1) {
        throw std::runtime_error(std::string(strerror(errno)));
    }

    freeaddrinfo(res);
    return fd;
}

int main(int argc, char **argv) {
    try {
        int sslVisionSocketDescriptor = getSslDescriptor();

        ros::init(argc, argv, sslConstants::SERVER_NODE_NAME);
        ros::NodeHandle n;
        ros::Publisher chatter_pub =
                n.advertise<std_msgs::String>(sslConstants::DATA_TOPIC_NAME, 1000);
        ros::Rate loop_rate(10);

        while (ros::ok()) {
            char buffer[1024];
            sockaddr_storage srcAddr;
            socklen_t srcAddrLen = sizeof(srcAddr);
            ssize_t count = recvfrom(sslVisionSocketDescriptor, buffer, sizeof(buffer), 0,
                                     (struct sockaddr *) &srcAddr, &srcAddrLen);
            if (count == -1) {
                throw std::runtime_error(std::string(strerror(errno)));
            } else if (count == sizeof(buffer)) {
                throw std::runtime_error("datagram too large for buffer: truncated");
            }

            std_msgs::String msg;
            std::stringstream ss;
            ss << buffer;
            msg.data = ss.str();

            ROS_INFO("%s", msg.data.c_str());
            chatter_pub.publish(msg);
            ros::spinOnce();

            loop_rate.sleep();
        }
    } catch(std::runtime_error& e) {
        std::cout << e.what() << "\n";
        return 1;
    }

    return 0;
}
