#QT      += core #gui
TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG -= debug_and_release
CONFIG += release
TARGET = ./bin/FingerprintExtractor

# include OpenCV 3.1 lib
OPENCV_PATH = C:/LIBs/OpenCV/3.1/build
INCLUDEPATH += $$OPENCV_PATH/include
LIBS += -L$$OPENCV_PATH/x64/vc12/lib/
LIBS += -lopencv_world310


FPEXTRACTOR_PATH = .
include(FingerprintExtractor.pri)

SOURCES += main.cpp

