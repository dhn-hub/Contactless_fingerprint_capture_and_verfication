#include "FingerprintVerification.h"


// Constructor
FingerprintVerification::FingerprintVerification(cv::Mat inFrame)
{
    mpFingerprintExtraction = new CFingerprintExtractor(inFrame);
}



FingerprintVerification::~FingerprintVerification()
{
    delete mpFingerprintExtraction;
}



void FingerprintVerification::newBatchInit(void)
{
    mFingerPalm.clear();
    mFingerList.clear();
    mFpqaList.clear();
    mIdxRankList.clear();
}



void FingerprintVerification::getFingerprints(cv::Mat inFrame, bool dbgFlag)
{
    mFingerPalm = mpFingerprintExtraction->doAllTheWork(inFrame); // mFingerList is updated
    if (mFingerPalm.size() > 0)
    {
        mFingerList.push_back( mFingerPalm[0] ); // mFingerPalm vector contains here only one lement (one finger)
    }
    if (dbgFlag)
    {
        cv::Mat inFrameResult = inFrame.clone();
        mpFingerprintExtraction->renderResults(inFrameResult, mFingerPalm);
        cv::imshow("results", inFrameResult);
    }

}



int FingerprintVerification::getNumFingerprints(void)
{
    return mFingerList.size();
}



SFPQA FingerprintVerification::qualityAssessment(bool dbgFlag)
{

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Calculate quality features
    ///
    for (int i=0; i<mFingerList.size(); i++)
    {
        if (mFingerList.size() > 0)
        {
            SFPQA fpqa = mQualityAssessment.sharpnessSobelGrad(mFingerList.at(i));
            mFpqaList.push_back(fpqa);
//            std::cout << "sharpness: " << fpqa.mSharpnessContrastSobelGrad << std::endl;
        }
    }
    ///
    /////////////////////////////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////////////////
    ///
    /// Sort the list
    ///
    // setup temporary list
    std::vector<float> unsortedListTmp;
    for (int i = 0; i<mFpqaList.size(); i++)
    {
        unsortedListTmp.push_back(mFpqaList.at(i).mSharpnessContrastSobelGrad);
    }
    if (unsortedListTmp.size() > 0)
    {
        mIdxRankList = mQualityAssessment.sortDesc(unsortedListTmp);
    }

    if (dbgFlag)
    {
        if (unsortedListTmp.size() > 0)
        {
            std::cout << "Max sharpness " << mFpqaList.at(mIdxRankList.at(0)).mSharpnessContrastSobelGrad << " at idx " << mIdxRankList.at(0) << "/" << mFpqaList.size()-1 << std::endl;
        }
        else
        {
            std::cout << "no fingerprint image." << std::endl;
        }

        for (int i = 0; i<mFpqaList.size(); i++)
        {
            SFPQA fpqa = mFpqaList.at(i);
            std::cout << "sharpness: " << fpqa.mSharpnessContrastSobelGrad << "  mIdxRankList: " << mIdxRankList.at(i) << std::endl;
        }
    }

    unsortedListTmp.clear();

    return mFpqaList.at(mIdxRankList.at(0));
}
