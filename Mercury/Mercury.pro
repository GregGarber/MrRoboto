QT += core network
QT -= gui

TARGET = Mercury
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    network.cpp \
    server.cpp

HEADERS += \
    network.h \
    server.h

