#QT      += core #gui
TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS += "-Wno-old-style-cast"

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG -= debug_and_release
CONFIG += release
TARGET = ./bin/FingerprintVerification


##################################################################################################
#
#                               PROJECT SETTINGS
#
# Compiler settings:
#       MSVC compiler 2013, v12, x64 build (no special version required)
#
# 3rd party libs:
#       OpenCV 3.1 (according to MSVC compiler version)
#
# OTHER:
#       FingerprintExtractor project is included into this project
#       (refer to FingerprintExtractor.pri)
#
##################################################################################################

include (opencv32_contrib_vc12.pri)

#INCLUDEPATH += /usr/local/include/opencv
#LIBS += -L/usr/local/lib -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dpm -lopencv_freetype -lopencv_fuzzy -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_rgbd -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_face -lopencv_plot -lopencv_dnn -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_xobjdetect -lopencv_objdetect -lopencv_ml -lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_photo -lopencv_imgproc -lopencv_core




# Include sources and headers of FingerprintExtravtor
FPEXTRACTOR_PATH = ../FingerprintExtractor
include($$FPEXTRACTOR_PATH/FingerprintExtractor.pri)

SOURCES += \
        main.cpp \
        FingerprintVerification.cpp \
        ./QualityAssessment/QualityAssessment.cpp \
        ./QualityAssessment/sobeledgestrength.cpp

HEADERS += \
        FingerprintVerification.h \
        ./QualityAssessment/QualityAssessment.h \
        ./QualityAssessment/sobeledgestrength.h \
        ./QualityAssessment/SFPQA.h \
    QualityAssessment/sobelEdgeStrength.h
