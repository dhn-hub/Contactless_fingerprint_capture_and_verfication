# FingerprintExtractor Module:
#
# FPEXTRACTOR_PATH is the relative path to the FingerprintExtractor module
#

INCLUDEPATH += $$FPEXTRACTOR_PATH

SOURCES += \
    $$FPEXTRACTOR_PATH/CImg.cpp \
    $$FPEXTRACTOR_PATH/CFingerprintExtractor.cpp \
    $$FPEXTRACTOR_PATH/EdgePair/CEdgeMatcher.cpp \
    $$FPEXTRACTOR_PATH/EdgePair/cvGeometricCalculations.cpp \
    $$FPEXTRACTOR_PATH/EdgePair/CvPointOperations.cpp \
    $$FPEXTRACTOR_PATH/PalmSegmentation/CPalmSegmentation.cpp \
    $$FPEXTRACTOR_PATH/PalmSegmentation/CBlobLabel.cpp \
    $$FPEXTRACTOR_PATH/PalmSegmentation/CFill.cpp


HEADERS += \
    $$FPEXTRACTOR_PATH/CImg.h \
    $$FPEXTRACTOR_PATH/SFinger.h \
    $$FPEXTRACTOR_PATH/CFingerprintExtractor.h \
    $$FPEXTRACTOR_PATH/EdgePair/CEdgeMatcher.h \
    $$FPEXTRACTOR_PATH/EdgePair/cvGeometricCalculations.h \
    $$FPEXTRACTOR_PATH/EdgePair/CvPointOperations.h \
    $$FPEXTRACTOR_PATH/EdgePair/myTypes.h \
    $$FPEXTRACTOR_PATH/PalmSegmentation/CPalmSegmentation.h \
    $$FPEXTRACTOR_PATH/PalmSegmentation/CBlobLabel.h \
    $$FPEXTRACTOR_PATH/PalmSegmentation/CFill.h
