#ifndef DISTENCODER_H
#define DISTENCODER_H

#include <trikControl/brickInterface.h>
#include<trikKernel/timeVal.h>
#include <trikControl/motorInterface.h>
#include <trikControl/encoderInterface.h>

class DistEncoder : public QObject {
    Q_OBJECT

public:
    DistEncoder(double configR, double configW, double configTurn);
    void startMove(int v);
    double distance();

public slots:
    void stop(int x);

private:

    double configR, configW, configTurn;

    trikControl::BrickInterface * brick;

    trikControl::MotorInterface * lMotor;
    trikControl::MotorInterface * rMotor;

    trikControl::EncoderInterface * encoder;
};

#endif // DISTENCODER_H
