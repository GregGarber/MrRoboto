QT += core serialport network
#remoteobjects
#QT -= gui

TARGET = BNO055
CONFIG += console
CONFIG -= app_bundle
LIBS += -lwiringPi
LIBS +=   -lqmqtt

TEMPLATE = app

SOURCES += main.cpp \
    bno055.cpp

HEADERS += \
    bno055.h
target.path = /home/pi/MrRoboto
target.files = BNO055
INSTALLS = target
#DISTFILES += bno055.rep
