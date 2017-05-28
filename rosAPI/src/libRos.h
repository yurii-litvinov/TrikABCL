#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include <vector>
#include "libRobot.h"
#include <cmath>

enum RESULT {
    OK ,
    NO_CONNECT ,
    NO_ARGUMENTS ,
    FAIL
};

float simulationTime=0.0;
unsigned int currentTime_updatedByTopicSubscriber=0;
struct timeval tv;
vector<Robot> robotVector;
int indexRobot = 0;
float accelX  = 0;
float accelY = 0;
float gyro = 0;
double angle = 0;
double positionRobot[3];

RESULT RosInit(std::string nodeName) {
    int _argc = 0;
    char** _argv = NULL;
    ros::init(_argc, _argv,nodeName.c_str());

    if(!ros::master::check())
        return NO_CONNECT;
    robotVector.push_back(Robot());
    return OK;
}

void simulationTimeCallback(const std_msgs::Float32& simTime) {
    simulationTime=simTime.data;
}

void sensorCallback(const std_msgs::Bool& sensTrigger) {
    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
    robotVector[indexRobot].sensorTrigger=sensTrigger.data;
}

void encoderLeftCallback(const std_msgs::Float32& encVal) {
    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
    robotVector[indexRobot].leftMotorEncoder = CalculationEncodert(robotVector[indexRobot].leftStepEncoder, encVal.data, robotVector[indexRobot].flagEncoder.first);
}
void encoderRightCallback(const std_msgs::Float32& encVal) {
    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
    robotVector[indexRobot].rightMotorEncoder = CalculationEncodert(robotVector[indexRobot].rightStepEncoder, encVal.data, robotVector[indexRobot].flagEncoder.second);
}

void accelCallback(const std_msgs::Float32MultiArray& accelVal) {

    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
    accelX += asin(accelVal.data[0]/ 9.8);
}

void gyroCallback(const std_msgs::Float32MultiArray& gyroVal) {

    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
    gyro = gyroVal.data[0];
}

void robotCallback(const std_msgs::Float32MultiArray& robotVal) {
    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
   positionRobot[0] = robotVal.data[0];
   positionRobot[1] = robotVal.data[1];
   positionRobot[2] = robotVal.data[2];
  // cout << "pos: " << positionRobot[0] << ' ' << positionRobot[1] << ' ' << positionRobot[2] << endl;
}

class Ros {
    public:
        int indRobot;
        Ros(int iR) : indRobot(iR) {}
        RESULT RosConfig(int argc, char * argv[]);
        RESULT RosStatus();


        void RosClose();
        void RosSubscribers(bool SubTime, bool SubSensor, bool SubLeftEncoder, bool SubRightEncoder);
        void RosPublishers(bool SubLeftMotor, bool SubRightMotor);
        void RosPublishMotorSpeed(float LeftMotorSpeed, float RightMotorSpeed);
        void RosSpinning(int mode);
        void RosRefresh();

    private:
        std::string leftMotorTopic;
        std::string rightMotorTopic;
        std::string sensorTopic;
        std::string simulationTimeTopic;
        std::string leftMotorEncoderTopic;
        std::string rightMotorEncoderTopic;
        std::string accelTopic;
        std::string gyroTopic;
        std::string robotTopic;

        ros::NodeHandle node;
        ros::Subscriber subSensor;
        ros::Subscriber subSimulationTime;
        ros::Subscriber subLeftEncoder;
        ros::Subscriber subRightEncoder;
        ros::Subscriber subAccel;
        ros::Subscriber subGyro;
        ros::Subscriber subRobot;

        ros::Publisher leftMotorSpeedPub;
        ros::Publisher rightMotorSpeedPub;
};

RESULT Ros::RosConfig(int argc, char * argv[]) {
    switch (argc) {
        case 9: {
            leftMotorTopic = "/" + (string) argv[1];
            rightMotorTopic = "/" + (string) argv[2];
            simulationTimeTopic = "/" + (string) argv[3];
            leftMotorEncoderTopic = "/" + (string) argv[4];
            rightMotorEncoderTopic = "/" + (string) argv[5];
            accelTopic = "/" + (string) argv[6];
            gyroTopic = "/" + (string) argv[7];
            robotTopic = "/" + (string) argv[8];
        }
        break;
        default: {
            return NO_ARGUMENTS;
        }
    }
    if (gettimeofday(&tv,NULL)==0)
        currentTime_updatedByTopicSubscriber=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;

    const ros::NodeHandle n ("~");
    node = n;

    return OK;
}

RESULT Ros::RosStatus() {
    return (ros::ok() ? OK : FAIL);
}

void Ros::RosClose() {
    ros::shutdown();
    printf("testRos just ended!\n");
}


void Ros::RosSubscribers(bool SubTime = false, bool SubSensor = false, bool SubLeftEncoder = true, bool SubRightEncoder = true){
    indexRobot = indRobot;
    if (SubTime) {
        subSimulationTime = node.subscribe(simulationTimeTopic.c_str(),1, simulationTimeCallback);
    }
    if (SubSensor) {
        subSensor=node.subscribe(sensorTopic.c_str(),1, sensorCallback);
    }
    if (SubLeftEncoder) {
        subLeftEncoder=node.subscribe(leftMotorEncoderTopic.c_str(),1, encoderLeftCallback);
    }
    if (SubRightEncoder) {
        subRightEncoder=node.subscribe(rightMotorEncoderTopic.c_str(),1,encoderRightCallback);
    }
    subAccel = node.subscribe(accelTopic.c_str(),1, accelCallback);
    subGyro = node.subscribe(gyroTopic.c_str(),1, gyroCallback);
    subRobot = node.subscribe(robotTopic.c_str(),1, robotCallback);
}

void Ros::RosPublishers(bool SubLeftMotor = false, bool SubRightMotor = false) {
    if (SubLeftMotor) {
        leftMotorSpeedPub = node.advertise<std_msgs::Float32>(leftMotorTopic.c_str(),1);
    }
    if (SubRightMotor) {
        rightMotorSpeedPub = node.advertise<std_msgs::Float32>(rightMotorTopic.c_str(),1);
    }
}

void Ros::RosPublishMotorSpeed(float LeftMotorSpeed, float RightMotorSpeed) {
    std_msgs::Float32 d;
    if (LeftMotorSpeed != FLT_MAX) {
        d.data = LeftMotorSpeed;
        leftMotorSpeedPub.publish(d);
    }
    if (RightMotorSpeed != FLT_MAX) {
        d.data = RightMotorSpeed;
        rightMotorSpeedPub.publish(d);
    }
}

void Ros::RosSpinning(int mode = 1) {
    switch(mode) {
    case 0:
        ros::spin();
        break;
    case 1:
        ros::spinOnce();
        break;
    }
}

