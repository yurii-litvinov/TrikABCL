#include <stdio.h>
#include <stdlib.h>
using namespace std;

const double pi = 3.141592653589793;

class Robot {

public:
    int leftStepEncoder;
    int rightStepEncoder;
    double leftMotorEncoder;
    double rightMotorEncoder;
    bool sensorTrigger;
    pair<int, int> flagEncoder;
    Robot() : leftStepEncoder(0), rightStepEncoder(0),
              leftMotorEncoder(0), rightMotorEncoder(0),
              sensorTrigger(false), flagEncoder(make_pair(0,0)) {}
    double CalculationEncodert(int & stepEnc, const double & val, int & flag);
};

double CalculationEncodert(int & stepEnc, const double & val, int& flag) {
    if (val > 0 && flag == 0) { flag = 1;}
    if (val < 0 && flag == 1) { flag = 2;}
    if (val > 0 && flag == 2) { ++stepEnc; flag = 0;}
    double d;
    d = (val >= 0 ? val / pi : (2*pi + val) / pi);
    return (d / 2 + stepEnc);
}
