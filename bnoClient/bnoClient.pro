QT += core network
QT -= gui

LIBS +=   -lqmqtt
CONFIG += c++11

TARGET = bnoClient
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    testit.cpp

HEADERS += \
    testit.h
