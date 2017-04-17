#include "libRos.h"

void RobotMove (int leftMotorSpeed, int rightMotorSpeed, Ros ros) {
    ros.RosPublishMotorSpeed(leftMotorSpeed, rightMotorSpeed);
}

void RobotMoveToPointForward(int leftMotorSpeed, int rightMotorSpeed, double xRobot, double yRobot, double X, double Y, Ros ros) {

double dist = sqrt((X-xRobot)*(X- xRobot) + (Y - yRobot)*(Y-yRobot));
   double R = 0.086;
   double currentDist = 0;
    while (currentDist < dist) {
        double doubleEncoders = robotVector[ros.indRobot].leftMotorEncoder + robotVector[ros.indRobot].rightMotorEncoder;
        doubleEncoders = doubleEncoders / (double)2;
        currentDist = doubleEncoders * pi * R;
        RobotMove(leftMotorSpeed, rightMotorSpeed, ros);
        ros.RosSpinning();
        usleep(5000);
    }
    RobotMove(0,0, ros);
    ros.RosSpinning();
    usleep(5000);
}
