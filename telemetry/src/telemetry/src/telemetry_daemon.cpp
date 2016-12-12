#include "ros/ros.h"

#include <QString>

#include <telemetry/Telemetry_data.h>

#include <trikControl/brickInterface.h>
#include <trikControl/brickFactory.h>
#include <trikControl/encoderInterface.h>
#include <trikControl/motorInterface.h>
#include <trikControl/batteryInterface.h>
#include <trikControl/sensorInterface.h>


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "telemetry_daemon");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<telemetry::Telemetry_data>("telemetry", 1000);

  ros::Rate loop_rate(10);

  trikControl::BrickInterface *brick = trikControl::BrickFactory::create();
  trikControl::VectorSensorInterface *accelerometer = brick->accelerometer();
  trikControl::VectorSensorInterface *gyroscope = brick->gyroscope();
  trikControl::BatteryInterface *battery = brick->battery();
  trikControl::EncoderInterface *encoder[4];
  trikControl::MotorInterface *motor[4];

  for (int i = 0; i < 4; ++i) {
    motor[i] = brick->motor(QString("M").append('0' + i + 1));
    encoder[i] = brick->encoder(QString("E").append('0' + i + 1));
  }

  while (ros::ok())
  {
    telemetry::Telemetry_data msg;

    QVector<int> accData = accelerometer->read();
    for (int i = 0; i < 3; i++) {
      msg.accelerometer[i] = accData[i];
    }

    QVector<int> gyrData = gyroscope->read();
    for (int i = 0; i < 3; i++) {
      msg.gyroscope[i] = accData[i];
    }

    msg.battery = battery->readVoltage();

    for (int i = 0; i < 4; ++i) {
      if (encoder[i]) {
        encoder[i]->reset();
        msg.encoder[i] = encoder[i]->readRawData();
      } else {
        msg.encoder[i] = -1;
        encoder[i] = brick->encoder(QString("E").append('0' + i + 1));
      }
    }

    for (int i = 0; i < 4; ++i) {

      if (motor[i]) {
        msg.powerMotor[i] = motor[i]->power();
      } else {
        msg.powerMotor[i] = -1;
        motor[i] = brick->motor(QString("M").append('0' + i + 1));
      }
    }

    ROS_INFO("A: %i %i %i, G: %i %i %i, B: %f, M1: %i, M2: %i, M3: %i, M4: %i, E1: %i, E2: %i, E3: %i, E4: %i", 
      msg.accelerometer[0], msg.accelerometer[1], msg.accelerometer[2], 
      msg.gyroscope[0], msg.gyroscope[1], msg.gyroscope[2], msg.battery,
      msg.powerMotor[0], msg.powerMotor[1], msg.powerMotor[2], msg.powerMotor[3], 
      msg.encoder[0], msg.encoder[1], msg.encoder[2], msg.encoder[3]);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
