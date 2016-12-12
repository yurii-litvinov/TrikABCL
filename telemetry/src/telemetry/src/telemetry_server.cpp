#include "ros/ros.h"
#include <telemetry/Telemetry_data.h>


void callback(const telemetry::Telemetry_data::ConstPtr& msg)
{
  ROS_INFO("A: %i %i %i, G: %i %i %i, B: %f, M1: %i, M2: %i, M3: %i, M4: %i, E1: %i, E2: %i, E3: %i, E4: %i", 
    msg->accelerometer[0], msg->accelerometer[1], msg->accelerometer[2], 
    msg->gyroscope[0], msg->gyroscope[1], msg->gyroscope[2], msg->battery,
    msg->powerMotor[0], msg->powerMotor[1], msg->powerMotor[2], msg->powerMotor[3], 
    msg->encoder[0], msg->encoder[1], msg->encoder[2], msg->encoder[3]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "telemetry_server");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("telemetry", 1000, callback);

  ros::spin();

  return 0;
}

