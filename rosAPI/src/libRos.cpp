#include "libRos.h"

RESULT Ros::RosConfig(int argc, char * argv[]) {
    switch (argc) {
        case 4: {
            leftMotorTopic = "/" + (string) argv[1];
            rightMotorTopic = "/" + (string) argv[2];
            simulationTimeTopic = "/" + (string) argv[3];
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


void Ros::RosSubscribers(bool SubTime, bool SubSensor){
    if (SubTime) {
        subSimulationTime = node.subscribe(simulationTimeTopic.c_str(),1,simulationTimeCallback);
    }
    if (SubSensor) {
        subSensor=node.subscribe(sensorTopic.c_str(),1,sensorCallback);
    }
}

void Ros::RosPublishers(bool SubLeftMotor, bool SubRightMotor) {
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

