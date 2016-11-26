#include "distencoder.h"
#include <trikControl/brickFactory.h>
#include <trikControl/motorInterface.h>
#include <trikControl/encoderInterface.h>
#include "math.h"
#include <trikControl/keysInterface.h>
#include <trikControl/displayInterface.h>
#include <string>
#include <QFile>
#include <QDebug>
#include <fstream>

DistEncoder::DistEncoder(double radius, double weight, double turn) {
    configR = radius;
    configW = weight;
    configTurn = turn;
    brick = trikControl::BrickFactory::create();
    lMotor = brick->motor("M4");
    rMotor = brick->motor("M3");

    encoder = brick->encoder("E4");
    encoder->reset();

    double d = encoder->readRawData();
    //std::ofstream res;
    //res.open("log.txt");
    qDebug() << "initial data" << d;
    //res.close();

    QObject::connect(brick->keys(), SIGNAL(buttonPressed(int,int)),this, SLOT(stop(int)));
}

void DistEncoder::startMove(int v) {
    lMotor->setPower(v);
    rMotor->setPower(v);
}

void DistEncoder::stop(int x) {
    if (x == 116) {
        lMotor->setPower(0);
        rMotor->setPower(0);
        double d = distance();
    //  std::ofstream res;
    //  res.open("log.txt");
        double enc = encoder->readRawData();
        qDebug() << abs(enc) << "distance" << d;
    //  res.close();
    }
}

double DistEncoder::distance() {
    int Turn = encoder->readRawData();
    return 2 * M_PI * configR * (abs(Turn) / configTurn);
}
