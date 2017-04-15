#include "libRos.h"
using namespace std;
int main(int argc,char* argv[])
{
    RESULT res = RosInit("testRos");
    if (res != OK) { cout << "NO_CONNECT" ; return 0; }
    Ros ros;
    res = ros.RosConfig(argc, argv);
    if (res != OK) { cout << "NO_ARGUMENT" ;  return 0; }

    ros.RosSubscribers(1, 0);

    ros.RosPublishers(1,1);

    float driveBackStartTime=-99.0f;
    unsigned int currentTime;
    if (gettimeofday(&tv,NULL)==0)
    {
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
        currentTime=currentTime_updatedByTopicSubscriber;
    }

    while (ros.RosStatus() == OK)
    {
       if (gettimeofday(&tv,NULL)==0)
        {
            currentTime=tv.tv_sec;
        }
        float desiredLeftMotorSpeed;
        float desiredRightMotorSpeed;
        desiredLeftMotorSpeed = 1.5;
        desiredRightMotorSpeed = 1.5;

        ros.RosPublishMotorSpeed(desiredLeftMotorSpeed, desiredRightMotorSpeed);

        ros::spinOnce();
        usleep(5000);
    }
    ros.RosClose();
    return(0);
}


