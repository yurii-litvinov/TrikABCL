#include "libMove.h"

using namespace std;

pair<double, double> encOld;
int main(int argc,char* argv[])
{
    RESULT res = RosInit("testRos");
    if (res != OK)  { cout << "NO_CONNECT" ; return 0; }
    Ros ros(0);
    res = ros.RosConfig(argc, argv);
    if (res  != OK) { cout << "NO_ARGUMENT" ;  return 0; }
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
    /*   RobotMoveToPoint(-0.318, 0.7, 0.089, 0.464, 1, ros,1);
       RobotMoveToPoint(-0.318, 0.7, -0.5, 0.5, 1, ros,1);
       RobotMoveToPoint( -0.318, 0.7, 0.0, 1.018, 2, ros,1);
       RobotMove(0,0, ros);
       RobotMoveToPoint( 0.0, 1.018, 0.5, 1.018, 2, ros,1);
       RobotMoveToPoint(-0.5, 0.5, -0.318, 0.7, 2, ros,1);
       RobotMoveToPoint(-1, -1, -0.318, 0.7 ,0.5, ros);
       RobotMove(0,0, ros);*/
       cout << "begin: " << endl;
        RobotMoveWithObstacle(-0.318, 0.7, 1.5, 0.7, 2, ros,0.086);
        RobotMove(0,0, ros);
       ros.RosSpinning();
       usleep(5000);
        cout << "stop";
        break;
    }
    ros.RosClose();
    return(0);
}


