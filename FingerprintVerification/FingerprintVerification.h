#ifndef FINGERPRINTVERIFICATION_H
#define FINGERPRINTVERIFICATION_H

#include <opencv2/opencv.hpp>

#include "CFingerprintExtractor.h"
#include "SFinger.h"
#include "QualityAssessment/QualityAssessment.h"
#include "QualityAssessment/SFPQA.h"

class FingerprintVerification
{
public:
    // Construtor
    FingerprintVerification(cv::Mat inFrame);
    ~FingerprintVerification();

    void newBatchInit(void);
    void getFingerprints(cv::Mat inFrame, bool dbgFlag=false);
    int getNumFingerprints(void);
    SFPQA qualityAssessment(bool dbgFlag=false);


    CFingerprintExtractor*  mpFingerprintExtraction;
    QualityAssessment       mQualityAssessment;

private:    
    std::vector<SFinger>    mFingerPalm;
    std::vector<SFinger>    mFingerList;
    std::vector<SFPQA>      mFpqaList;
    std::vector<int>        mIdxRankList;

};

#endif // FINGERPRINTVERIFICATION_H
