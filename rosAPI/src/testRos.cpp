#include "libMove.h"

using namespace std;

pair<double, double> encOld;
int main(int argc,char* argv[])
{
    RESULT res = RosInit("testRos");
    if (res != OK) { cout << "NO_CONNECT" ; return 0; }
    Ros ros(0);
    res = ros.RosConfig(argc, argv);
    if (res != OK) { cout << "NO_ARGUMENT" ;  return 0; }
    cout << argv << endl;

    ros.RosSubscribers(1, 0, 1);
    ros.RosPublishers(1,1);

    float driveBackStartTime=-99.0f;
    unsigned int currentTime;
    if (gettimeofday(&tv,NULL)==0)
    {
        currentTime_updatedByTopicSubscriber=tv.tv_sec;
        currentTime=currentTime_updatedByTopicSubscriber;
    }
int flag = 0;
    while (ros.RosStatus() == OK)
    {
       if (gettimeofday(&tv,NULL)==0)
        {
            currentTime=tv.tv_sec;
        }
        float leftMotorSpeed = 1.0;
        float rightMotorSpeed = 1.0;
        RobotMove(leftMotorSpeed, rightMotorSpeed, ros);
       if (flag == 0 ) {
       RobotMoveToPointForward(1,1, -0.318, 0.7, 0.2, 0.7, ros);
       flag  = 1;
       }
       if (flag == 1) break;
    }
    ros.RosClose();
    return(0);
}


