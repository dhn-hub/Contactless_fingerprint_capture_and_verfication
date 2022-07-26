#ifndef _CFingerprintExtractor_H_
#define _CFingerprintExtractor_H_

#include <opencv2/opencv.hpp>
#include "SFinger.h"

#include "EdgePair/CEdgeMatcher.h"
#include "PalmSegmentation/CPalmSegmentation.h"
#include "CImg.h"

class CFingerprintExtractor
{
public:
    CFingerprintExtractor(cv::Mat inFrame);

    std::vector<SFinger> doAllTheWork(cv::Mat inFrame);
    void renderResults(Mat &inFrameResult, std::vector<SFinger> fingersPalm);

private:

    double  mScaleFactor;
    int     mNumMaxFinger;
    double  mMaxAngleFinger;
    double  mOffsetLenForWidth;
    int     mMaxIterFingerWidth;
    double  mRatioFingerHeight2Width;
    int     mMinFingerWidth;
    int     mMaxFingerWidth;

    cv::Mat mInFrameScaled;

    CPalmSegmentation   mPalmSegmentation;
    CEdgeMatcher*       mpEdgeMatcher;

};



#endif   // _CFingerprintExtractor_H_
