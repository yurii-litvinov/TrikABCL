#include "libRos.h"
#include <vector>
enum turn {
    LEFT, RIGHT
};

class Obstacle {
public:
    double xDownLeft, xUpLeft, xDownRight, xUpRight;
    double yDownLeft, yUpLeft, yDownRight, yUpRight;
   public:
    Obstacle(double xDl,double yDl,double xUl,double yUl,double xDr,double yDr,double xUr,double yUr) : xDownLeft(xDl), yDownLeft(yDl),
                                                                                xUpLeft(xUl), yUpLeft(yUl),
                                                                                xDownRight(xDr), yDownRight(yDr),
                                                                                xUpRight(xUr), yUpRight(yUr) {}

};

vector<Obstacle> mapObstacle;

double startAngle = 0;
void RobotMove (double leftMotorSpeed, double rightMotorSpeed, Ros ros) {
    ros.RosPublishMotorSpeed(leftMotorSpeed, rightMotorSpeed);
    ros.RosSpinning();
    usleep(5000);
}

void RobotMoveToPointForward(double leftMotorSpeed, double rightMotorSpeed, double xRobot, double yRobot, double X, double Y, Ros ros) {
double startEnc = robotVector[ros.indRobot].leftMotorEncoder + robotVector[ros.indRobot].rightMotorEncoder;
double dist = sqrt((X-xRobot)*(X- xRobot) + (Y - yRobot)*(Y-yRobot));
   double R = 0.086;
   cout << xRobot << ' ' << yRobot << ' ' << X << ' ' << Y << ' ' << dist << endl;
   double currentDist = 0;
    while (currentDist < dist) {
        double doubleEncoders = robotVector[ros.indRobot].leftMotorEncoder + robotVector[ros.indRobot].rightMotorEncoder - startEnc;
        doubleEncoders = doubleEncoders / (double)2;
        currentDist = doubleEncoders * pi * R;
        RobotMove(leftMotorSpeed, rightMotorSpeed, ros);

    }
    RobotMove(0,0, ros);
}

void RobotTurn(double angle, Ros ros, turn t , double MotorSpeed = 2) {
    double R = 0.062;
    double k = 0.95; //2 - for diagonal rotation
    double leftMotorSpeed = MotorSpeed * (t == LEFT ? -1 : 1);
    double rightMotorSpeed = MotorSpeed * (t == RIGHT ? -1 : 1);
    double l = k * 3.14 * R * angle / 180 ;
    double currentDist = 0;
    double startEncLeft = robotVector[ros.indRobot].leftMotorEncoder;
    double startEncRight = robotVector[ros.indRobot].rightMotorEncoder;
    while (currentDist < l) {
        RobotMove(leftMotorSpeed, rightMotorSpeed, ros);
         double doubleEncoders = (t == LEFT ? robotVector[ros.indRobot].rightMotorEncoder - startEncRight : robotVector[ros.indRobot].leftMotorEncoder - startEncLeft);
         currentDist = doubleEncoders * pi * R;
        ros.RosSpinning();
        usleep(5000);
    }
}

void RobotMoveToPoint(double xRobot, double yRobot,double toX, double toY, double speed, Ros ros, bool fl) {
    double angle = 0;
    double delta = 0.1;
    if (abs(yRobot - toY) < delta) {
        angle = (xRobot > toX ? 180 : 0);
        angle -= startAngle;
        //cout << "X: " << angle << endl;
    }
    else
    if (abs(xRobot - toX) < delta) {
        angle = (yRobot > toY ? 90 : 270);
        angle -= startAngle;
    }
    else {
        angle = atan(abs(yRobot - toY) / abs(xRobot - toX)) * 180 / 3.14;
        angle = 90 - angle;
        //cout << "1: " << angle << ' ';

        if (angle == startAngle) {
            angle = 180;
        }
            else {
            if (toX > xRobot && toY > yRobot) angle += 270; else
            if (toX < xRobot && toY > yRobot) angle += 180; else
            if (toX < xRobot && toY < yRobot) angle += 90;
            angle -= startAngle;
            }
        }
    if (angle  > 360 ) angle -= 360;
    startAngle += (angle);
    if (angle < 0) angle +=360;
    //cout << "fffff" << angle << ' ' << startAngle << endl;
    if (abs(angle) <= 180)
    RobotTurn(abs(angle), ros, RIGHT, speed );
    else {
        //cout << 360 - abs(angle) << endl;
        RobotTurn(360 - abs(angle), ros, LEFT, speed );
    }
   if (fl) {
    RobotMoveToPointForward(speed, speed, xRobot, yRobot, toX, toY, ros);
    }
}

void makeMap(vector<Obstacle> &mapObstacle, double R) {
    Obstacle obj (0.175 - R, 0.85 + R, 0.475 + R, 0.85 + R, 0.175 - R, 0.55 - R, 0.475 + R, 0.55 - R);
    mapObstacle.push_back(obj);
    Obstacle obj1 (0.85 - R, 0.9 + R, 1.15 + R, 0.9 + R, 0.85 - R, 0.6 - R, 1.15 + R, 0.6 - R);
    mapObstacle.push_back(obj1);

}
double distance(double x1, double y1, double x2, double y2) {
    return (x1-x2)*(x1-x2) + (y1-y2) * (y1-y2);
}
void makeToPoint(pair<double, double> & point, double x, double y, double & dist, bool & flag) {
    double xRobot = positionRobot[0];
    double yRobot = positionRobot[1];
    double e = 0.35;
    double currentDist = distance(xRobot, yRobot, x, y);

    if (currentDist < e*e && currentDist < dist) {
        dist = currentDist;
        point = make_pair(x, y);
        flag = true;
    }
}

void RobotMoveWithObstacle(double xRobot, double yRobot,double toX, double toY, double speed, Ros ros, double R) {
    makeMap(mapObstacle, R);
    bool fl = 0;
    double globalDist = distance(xRobot, yRobot, toX, toY);
    while (globalDist > 0.3 * 0.3  && !fl) {
        pair<double, double> point = make_pair(toX, toY);
        //cout << "bGl " << globalDist << endl;
        //cout << xRobot << ' ' << yRobot<< ' ' << toX << ' ' << toY << endl;
        RobotMoveToPoint(xRobot, yRobot, toX, toY, 2, ros, 0);
        RobotMove(0,0, ros);
        ros.RosSpinning();
        usleep(5000);
        bool flag = false;
        if (mapObstacle.size() == 0 ) fl = true;
        while (!flag && !fl) {
            RobotMove(2, 2, ros);
            xRobot = positionRobot[0];
            yRobot = positionRobot[1];
            double dist = distance(xRobot, yRobot, toX, toY);

            for (int i = 0; i < mapObstacle.size(); ++i) {
                makeToPoint(point, mapObstacle[i].xDownLeft, mapObstacle[i].yDownLeft, dist, flag);
                makeToPoint(point, mapObstacle[i].xDownRight, mapObstacle[i].yDownRight, dist, flag);
                makeToPoint(point, mapObstacle[i].xUpLeft, mapObstacle[i].yUpLeft, dist, flag);
                makeToPoint(point, mapObstacle[i].xUpRight, mapObstacle[i].yUpRight, dist, flag);
                if (flag) {
                    mapObstacle.erase(mapObstacle.begin() + i);
                    break;
                }
            }
        }
        RobotMove(0,0, ros);
        ros.RosSpinning();
        usleep(5000);

        RobotMoveToPoint(xRobot, yRobot, point.first, point.second, 2, ros, 1);
        xRobot = positionRobot[0];
        yRobot = positionRobot[1];
        if (!fl) {
            RobotMoveToPoint(xRobot, yRobot, point.first + 0.3 + 2*R, point.second, 2  , ros, 1);
        }
        xRobot = positionRobot[0];
        yRobot = positionRobot[1];
        globalDist = distance(xRobot, yRobot, toX, toY);
        //cout << xRobot << ' ' << yRobot << ' '<< toX << ' ' << toY << endl;
        //cout << "gl " << globalDist << endl;
      }

    }
