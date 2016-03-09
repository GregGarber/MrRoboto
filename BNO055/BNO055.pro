QT += core serialport
QT -= gui

TARGET = BNO055
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    bno055.cpp

HEADERS += \
    bno055.h

