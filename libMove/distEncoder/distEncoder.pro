#-------------------------------------------------
#
# Project created by QtCreator 2016-10-28T00:15:18
#
#-------------------------------------------------

QT       -= gui

TARGET = distEncoder
TEMPLATE = lib
CONFIG += staticlib

SOURCES += main.cpp \
    distencoder.cpp \


HEADERS += distencoder.h
unix {
    target.path = /usr/lib
    INSTALLS += target
}

TEMPLATE = app

CONFIG += console
QMAKE_CXXFLAGS += -std=c++11

DESTDIR = .


TRIK_RUNTIME_DIR = ../../

include(../../trikControl/trikControlExport.pri)
