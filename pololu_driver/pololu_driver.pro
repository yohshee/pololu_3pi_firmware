#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:59:54
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pololu_driver
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    unixserial.cpp \
    robotaction.cpp \
    pololurobot.cpp

HEADERS  += mainwindow.h \
    unixserial.h \
    robotaction.h \
    pololurobot.h

FORMS    += mainwindow.ui
